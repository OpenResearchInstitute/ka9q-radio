// $Id$

// General purpose linear modes demodulator
// Derived from dsb.c by folding in ISB and making coherent tracking optional
// Sept 20 2017 Phil Karn, KA9Q

#define _GNU_SOURCE 1
#include <complex.h>
#include <math.h>
#include <fftw3.h>
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>


#include "dsp.h"
#include "filter.h"
#include "radio.h"
#include "audio.h"

// Configurable parameters

static float const hangtime = 1.1;      // AGC Hang for 1.1 seconds after new peak
static float const recovery_rate = 6;   // AGC recovery rate 6 db/sec after hang finishes
static float const snrthreshdb = 3;     // Loop lock threshold at +3 dB SNR
static int   const fftsize = 1 << 16;   // search FFT bin size = 64K = 1.37 sec @ 48 kHz
static float const damping = M_SQRT1_2; // PLL loop damping factor; 1/sqrt(2) is "critical" damping
static float const searchhigh = 1000;   // FFT search limits
static float const searchlow = -1000;
static float const unlock_time = 1;     // Search 1 sec after loss of lock


void *demod_dsb(void *arg){
  pthread_setname("dsb");
  assert(arg != NULL);
  struct demod * const demod = arg;

  // Set derived (and other) constants
  float const samptime = demod->decimate / demod->samprate;  // Time between (decimated) samples
  float const snrthresh = powf(10,snrthreshdb/10);           // SNR threshold for lock
  // FFT search params
  int   const unlock_limit = unlock_time / samptime;        // Start acquiring after loss of lock
  float const binsize = 1 / (fftsize * samptime);           // FFT bin size, Hz
  int   const lowlimit = round(searchlow / binsize);        // FFT bin indices for search limits
  int   const highlimit = round(searchhigh / binsize);

  // Second-order PLL loop filter (see Gardner)
  float const phase_scale = 2 * M_PI * samptime;           // Convert hertz to radians/sample
  float const vcogain = 2*M_PI;                            // 1 Hz = 2pi radians/sec per "volt"
  float const pdgain = 1;                                  // "volts" per radian (unity)
  float const natfreq = binsize * 2*M_PI;                  // loop natural frequency in rad/sec
  float const tau1 = vcogain * pdgain / (natfreq*natfreq); // 1 / 2pi
  float const integrator_gain = 1 / tau1;                  // 2pi
  float const tau2 = 2 * damping / natfreq;                // sqrt(2) / 2pi = 1/ (pi*sqrt(2))
  float const prop_gain = tau2 / tau1;                     // sqrt(2)/2
  float const ramprate = binsize * samptime / integrator_gain;   // sweep at one bin/sec

  // AGC
  int hangcount = 0;
  float const agcratio = dB2voltage(recovery_rate * ((float)demod->L/demod->samprate)); // 6 dB/sec
  int const hangmax = hangtime * (demod->samprate/demod->L); // 1.1 second hang before gain increase

  demod->gain = dB2voltage(70.);
  demod->snr = -INFINITY;

  // Detection filter
  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,
					       (demod->flags & CONJ) ? CROSS_CONJ : COMPLEX);
  demod->filter = filter;
  set_filter(filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);

  // Search FFT
  complex float * const fftinbuf = fftwf_alloc_complex(fftsize);
  complex float * const fftoutbuf = fftwf_alloc_complex(fftsize);  
  fftwf_plan fft_plan = fftwf_plan_dft_1d(fftsize,fftinbuf,fftoutbuf,FFTW_FORWARD,FFTW_ESTIMATE);
  int fft_ptr = 0;

  // Initialize PLL
  complex float fine_phasor = 1;        // fine offset LO
  complex float coarse_phasor = 1;      // FFT-controlled offset LO
  complex float coarse_phasor_step = 1; // 0 Hz to start
  float integrator = 0;                 // 2nd order loop integrator
  float delta_f = 0;                    // FFT-derived offset
  float ramp = 0;                       // Frequency sweep (do we still need this?)
  int unlock_time = 0;                  // Time since apparent loss of lock, samples
  float calibrate_offset = 0;           // Frequency error for calibration mode

  while(!demod->terminate){
    // New samples
    fillbuf(demod,filter->input.c,filter->ilen);
    demod->second_LO_phasor = spindown(demod,filter->input.c);
    demod->if_power = cpower(filter->input.c,filter->ilen);
    execute_filter(filter);
    if(!isnan(demod->n0))
      demod->n0 += .01 * (compute_n0(demod) - demod->n0);
    else
      demod->n0 = compute_n0(demod); // Happens at startup

    if(demod->flags & COHERENT){
      // Copy into circular input buffer for FFT in case we need it
      if(demod->flags & SQUARE){
	for(int i=0;i<filter->olen;i++)
	  fftinbuf[fft_ptr+i] = filter->output.c[i] * filter->output.c[i];
      } else {
	memcpy(fftinbuf+fft_ptr,filter->output.c,filter->olen*sizeof(*filter->output.c));
      }
      fft_ptr = (fft_ptr + filter->olen) % fftsize; // assumes filter->olen divides fftsize, otherwise we'll have a buffer overrun!
      
      // If SNR is below threshold, use FFT to find approximate carrier frequency
      if(demod->snr < snrthresh){
	unlock_time += filter->olen;
	if(unlock_time >= unlock_limit){
	  // Run FFT, look for peak bin
	  fftwf_execute(fft_plan);
	  // Search limited range of FFT buffer for peak energy
	  int maxbin = 0;
	  float maxenergy = 0;
	  
	  for(int n = lowlimit; n <= highlimit; n++){
	    float const e = cnrmf(fftoutbuf[(n + fftsize) % fftsize]);
	    if(e > maxenergy){
	      maxenergy = e;
	      maxbin = n;
	    }
	  }
	  
	  delta_f = binsize * maxbin; // cycles per second	
	  if(demod->flags & SQUARE)
	    delta_f /= 2; // Squaring loop provides 2x frequency
	  
	  coarse_phasor_step = csincos(-phase_scale * delta_f);
	  if(ramp == 0) // not already sweeping
	    ramp = ramprate;
	}
      } else {
	unlock_time = 0; // Reset
	ramp = 0;
      }
      
      // Apply coarse offset, determine fine frequency adjustment with PLL
      complex float accum = 0;
      for(int n=0;n<filter->olen;n++){
	filter->output.c[n] *= coarse_phasor * fine_phasor;
	float carrier_phase;
	if(demod->flags & SQUARE){
	  complex float const ss = filter->output.c[n] * filter->output.c[n];
	  accum += ss;
	  //carrier_phase = fast_cargf(ss)/2;
	  carrier_phase = cargf(ss)/2;
	} else {
	  accum += filter->output.c[n];
	  // Fast (?) approximation to atan2/carg
	  //carrier_phase = fast_cargf(filter->output.c[n]);
	  carrier_phase = cargf(filter->output.c[n]);
	}	
	// Lag-lead (integral plus proportional)
	integrator += carrier_phase * samptime + ramp;
	// Acquisition frequency sweep between +/- binsize/2
	if((integrator * integrator_gain >= binsize/2) && (ramp > 0))
	  ramp = -ramprate; // reached sweep limit, toggle ramp direction
	else if((integrator * integrator_gain <= -binsize/2) && (ramp < 0))
	  ramp = ramprate;
	
	float const feedback = integrator_gain * integrator + prop_gain * carrier_phase; // units of Hz
	// Small angle approximation to csincosf(-phase_scale * feedback); 
	complex float const fine_phasor_step = CMPLXF(1,-phase_scale * feedback); 
	
	if((demod->flags & CAL) && demod->snr >= snrthresh){
	  // In calibrate mode, keep highly smoothed estimate of frequency offset
	  calibrate_offset += .00001 * (feedback + delta_f - calibrate_offset);
	}
	demod->foffset += .001 * (feedback + delta_f - demod->foffset);
	
	coarse_phasor *= coarse_phasor_step;
	fine_phasor *= fine_phasor_step;
      }
      if(demod->flags & SQUARE)
	demod->cphase = cargf(accum)/2;
      else
	demod->cphase = cargf(accum);
      
      fine_phasor /= cabsf(fine_phasor);
      coarse_phasor /= cabsf(coarse_phasor);
    }      

    // compute signal (I) and noise (Q) levels, total level and SNR
    float totampl;
    {
      complex float const p = cpowers(filter->output.c,filter->olen);
      float const signal = crealf(p);
      float const noise = cimagf(p);
      if(isnan(signal) || isnan(noise)){
	// Startup transient
	demod->bb_power = 0;
	demod->snr = -INFINITY;
	totampl = 0;
      } else {
	demod->snr = (signal / noise) - 1; // S/N as power ratio
	demod->bb_power = signal + noise;
	totampl = sqrtf(signal + noise);
      }
    }
    if((demod->flags & CAL) && demod->snr >= snrthresh){
      // In calibrate mode, apply and clear the current measured offset
      set_cal(demod,demod->calibrate - calibrate_offset/demod->frequency);
      calibrate_offset = 0;
    }

    // Manual frequency shift
    if(demod->shift != 0){
      for(int n=0; n < filter->olen; n++){
	filter->output.c[n] *= demod->shift_phasor;
	demod->shift_phasor *= demod->shift_phasor_step;
      }
      demod->shift_phasor /= cabs(demod->shift_phasor);
    }
    // Q is on the right channel, so use both I and Q for gain setting so we don't blast our ears when the phase is wrong
    if(demod->gain * totampl > demod->headroom){ // Target to about -10 dBFS
      // New signal peak: decrease gain and inhibit re-increase for a while
      if(totampl != 0)
	demod->gain = demod->headroom / totampl;
      hangcount = hangmax;
    } else {
      // Not a new peak, but the AGC is still hanging at the last peak
      if(hangcount != 0){
	hangcount--;
      } else {
	// OK to increase gain; should enforce a limit
	demod->gain *= agcratio;
      }
    }
    send_stereo_audio(demod->audio,filter->output.c,filter->olen,demod->gain);
  }
  fftwf_free(fftinbuf);
  fftwf_free(fftoutbuf);  
  fftwf_destroy_plan(fft_plan);
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
