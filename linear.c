// $Id: linear.c,v 1.28 2018/12/02 09:16:45 karn Exp karn $

// General purpose linear demodulator
// Handles USB/IQ/CW/etc, basically all modes but FM and envelope-detected AM
// Copyright Sept 20 2017 Phil Karn, KA9Q

#define _GNU_SOURCE 1
#include <assert.h>
#include <complex.h>
#include <math.h>
#include <fftw3.h>
#include <pthread.h>
#include <string.h>

#include "misc.h"
#include "dsp.h"
#include "filter.h"
#include "radio.h"


void *demod_linear(void *arg){
  pthread_setname("linear");
  assert(arg != NULL);
  struct demod * const demod = arg;

  demod->loop_bw = 1; // eventually to be set from mode table

  // Set derived (and other) constants
  float const samptime = (float)demod->filter.decimate / demod->input.samprate;  // Time between (decimated) samples
  float const blocktime = samptime * demod->filter.L; // Update rate of fine PLL (once/block)

  // AGC
  int hangcount = 0;
  float const recovery_factor = dB2voltage(demod->agc.recovery_rate * samptime); // AGC ramp-up rate/sample
#if 0
  float const attack_factor = dB2voltage(demod->agc.attack_rate * samptime);      // AGC ramp-down rate/sample
#endif
  int const hangmax = demod->agc.hangtime / samptime; // samples before AGC increase
  if(isnan(demod->agc.gain))
    demod->agc.gain = dB2voltage(100.0); // initial setting

  // Coherent mode parameters
  float const snrthreshdb = 3;     // Loop lock threshold at +3 dB SNR
  int   const fftsize = 1 << 16;   // search FFT bin size = 64K = 1.37 sec @ 48 kHz
  float const damping = M_SQRT1_2; // PLL loop damping factor; 1/sqrt(2) is "critical" damping
  float const lock_time = 1;       // hysteresis parameter: 2*locktime seconds of good signal -> lock, 2*locktime sec of bad signal -> unlock
  int   const fft_enable = 1;

  // FFT search params
  float const snrthresh = powf(10,snrthreshdb/10);          // SNR threshold for lock
  int   const lock_limit = round(lock_time / samptime);            // Stop sweeping after locked for this amount of time
  float const binsize = 1. / (fftsize * samptime);          // FFT bin size, Hz
  // FFT bin indices for search limits. Squaring doubles frequency, so double the search range
  float const searchhigh = 300;    // FFT search limits, in Hz
  float const searchlow =  -300;
  int   const lowlimit =  round((demod->square ? 2 : 1) * searchlow / binsize);
  int   const highlimit = round((demod->square ? 2 : 1) * searchhigh / binsize);

  // Second-order PLL loop filter (see Gardner)
  float const phase_scale = samptime;           // cycles/sample (previous radians/sample)
  float const vcogain = 2*M_PI;                            // 1 Hz = 2pi radians/sec per "volt"
  float const pdgain = 1;                                  // phase detector gain "volts" per radian (unity from atan2)
  float const natfreq = demod->loop_bw * 2*M_PI;                  // loop natural frequency in rad/sec
  float const tau1 = vcogain * pdgain / (natfreq*natfreq); // 1 / 2pi
  float const integrator_gain = 1 / tau1;                  // 2pi
  float const tau2 = 2 * damping / natfreq;                // sqrt(2) / 2pi = 1/ (pi*sqrt(2))
  float const prop_gain = tau2 / tau1;                     // sqrt(2)/2
  //  float const ramprate = demod->loop_bw * blocktime / integrator_gain;   // sweep at one loop bw/sec
  float const ramprate = 0; // temp disable

#if 0
  // DC removal from envelope-detected AM and coherent AM
  complex float DC_filter = 0;
  float const DC_filter_coeff = .0001;
#endif

  demod->snr = 0;

  // Detection filter
  struct filter_out * const filter = create_filter_output(demod->filter.in,NULL,demod->filter.decimate,
					       (demod->filter.isb) ? CROSS_CONJ : COMPLEX);
  demod->filter.out = filter;
  set_filter(filter,samptime*demod->filter.low,samptime*demod->filter.high,demod->filter.kaiser_beta);

  // Carrier search FFT
  complex float * fftinbuf = NULL;
  complex float *fftoutbuf = NULL;
  fftwf_plan fft_plan = NULL;
  int fft_ptr = 0;  

  if(fft_enable){
    fftinbuf = fftwf_alloc_complex(fftsize);
    fftoutbuf = fftwf_alloc_complex(fftsize);  
    fft_plan = fftwf_plan_dft_1d(fftsize,fftinbuf,fftoutbuf,FFTW_FORWARD,FFTW_ESTIMATE);
  }

  // PLL oscillator is in two parts, coarse and fine, so that small angle approximations
  // can be used to rapidly tweak the frequency by small amounts
  struct osc fine;
  memset(&fine,0,sizeof(fine));
  set_osc(&fine, 0.0, 0.0);

  struct osc coarse;                    // FFT-controlled offset LO
  memset(&coarse,0,sizeof(coarse));
  set_osc(&coarse,0.0, 0.0);            // 0 Hz to start
  
  float integrator = 0;                 // 2nd order loop integrator
  float delta_f = 0;                    // FFT-derived offset
  float ramp = 0;                       // Frequency sweep (do we still need this?)
  int lock_count = 0;

  while(!demod->terminate){
    // New samples
    // Copy ISB flag to filter, since it might change
    if(demod->filter.isb)
      filter->out_type = CROSS_CONJ;
    else
      filter->out_type = COMPLEX;

    execute_filter_output(filter);    
    if(!isnan(demod->n0))
      demod->n0 += .001 * (compute_n0(demod) - demod->n0);
    else
      demod->n0 = compute_n0(demod); // Happens at startup

    // Carrier (or regenerated carrier) tracking in coherent mode
    if(demod->pll){
      // Copy into circular input buffer for FFT in case we need it for acquisition
      if(fft_enable){
	if(demod->square){
	  // Squaring loop is enabled; square samples to strip BPSK or DSB modulation
	  // and form a carrier component at 2x its actual frequency
	  // This is of course suboptimal for BPSK since there's no matched filter,
	  // but it may be useful in a pinch
	  for(int i=0;i<filter->olen;i++){
	    fftinbuf[fft_ptr++] = filter->output.c[i] * filter->output.c[i];
	    if(fft_ptr >= fftsize)
	      fft_ptr -= fftsize;
	  }
	} else {
	  // No squaring, just analyze the samples directly for a carrier
	  for(int i=0;i<filter->olen;i++){
	    fftinbuf[fft_ptr++] = filter->output.c[i];
	    if(fft_ptr >= fftsize)
	      fft_ptr -= fftsize;
	  }
	}
      }
      // Loop lock detector with hysteresis
      // If the loop is locked, the SNR must fall below the threshold for a while
      // before we declare it unlocked, and vice versa
      if(demod->snr < snrthresh){
	lock_count -= filter->olen;
      } else {
	lock_count += filter->olen;
      }
      if(lock_count >= lock_limit){
	lock_count = lock_limit;
	demod->pll_lock = 1;
      }
      if(lock_count <= -lock_limit){
	lock_count = -lock_limit;
	demod->pll_lock = 0;
      }
      demod->lock_timer = lock_count;

      // If loop is out of lock, reacquire
      if(!demod->pll_lock){
	if(fft_enable){
	  // Run FFT, look for peak bin
	  // Do this every time??
	  fftwf_execute(fft_plan);
	  
	  // Search limited range of FFT buffer for peak energy
	  int maxbin = 0;
	  float maxenergy = 0;
	  for(int n = lowlimit; n <= highlimit; n++){
	    float const e = cnrmf(fftoutbuf[n < 0 ? n + fftsize : n]);
	    if(e > maxenergy){
	      maxenergy = e;
	      maxbin = n;
	    }
	  }
	  double new_delta_f = binsize * maxbin;
	  if(demod->square)
	    new_delta_f /= 2; // Squaring loop provides 2xf component, so we must divide by 2
	  
	  if(new_delta_f != delta_f){
	    delta_f = new_delta_f;
	    integrator = 0; // reset integrator
	    set_osc(&coarse, -phase_scale*delta_f, 0.0);
	  }
	}
	if(ramp == 0) // not already sweeping
	  ramp = ramprate;
      } else { // !pll_lock
	ramp = 0;
      }
      // Apply coarse and fine offsets, gather DC phase information
      complex float accum = 0;
      for(int n=0;n<filter->olen;n++){
	filter->output.c[n] *= step_osc(&coarse) * step_osc(&fine);

	complex float ss = filter->output.c[n];
	if(demod->square)
	  ss *= ss;
	
	accum += ss;
      }
      demod->cphase = cargf(accum);
      if(demod->square)
	demod->cphase /= 2; // Squaring doubles the phase


      // fine PLL on block basis
      // Includes ramp generator for frequency sweeping during acquisition
      float carrier_phase = demod->cphase;

      // Lag-lead (integral plus proportional) 
      integrator += carrier_phase * blocktime + ramp;
      float const feedback = integrator_gain * integrator + prop_gain * carrier_phase; // units of Hz
      if(fabsf(feedback * phase_scale) < .01)
	fine.phasor_step = CMPLXF(1,-2*M_PI*phase_scale * feedback);  // Small angle approximation
      else
	fine.phasor_step = csincosf(-2*M_PI*phase_scale * feedback); 
      
      // Acquisition frequency sweep
      if((feedback >= binsize) && (ramp > 0))
	ramp = -ramprate; // reached upward sweep limit, sweep down
      else if((feedback <= binsize) && (ramp < 0))
	ramp = ramprate;  // Reached downward sweep limit, sweep up
      
      if(isnan(demod->foffset))
	demod->foffset = feedback + delta_f;
      else
	demod->foffset += 0.001 * (feedback + delta_f - demod->foffset);
    }
    // Demodulation
    float signal = 0;
    float noise = 0;
    
    for(int n=0; n<filter->olen; n++){
      // Assume signal on I channel, so only noise on Q channel
      // True only in coherent modes when locked, but we'll need total power anyway
      complex float s = filter->output.c[n];
      float rp = crealf(s) * crealf(s);
      float ip = cimagf(s) * cimagf(s);
      signal += rp;
      noise += ip;

      float amplitude = sqrtf(rp + ip);
      
      // AGC
      // Lots of people seem to have strong opinions how AGCs should work
      // so there's probably a lot of work to do here
      // The attack_factor feature doesn't seem to work well; if it's at all
      // slow you get an annoying "pumping" effect.
      // But if it's too fast, brief spikes can deafen you for some time
      // What to do?
      if(isnan(demod->agc.gain)){
	demod->agc.gain = demod->agc.headroom / amplitude; // Startup
      } else if(amplitude * demod->agc.gain > demod->agc.headroom){
	demod->agc.gain = demod->agc.headroom / amplitude;
	//	  demod->agc.gain *= attack_factor;
	hangcount = hangmax;
      } else if(hangcount != 0){
	hangcount--;
      } else {
	demod->agc.gain *= recovery_factor;
      }
      filter->output.c[n] *= demod->agc.gain;
    }
    // Optional frequency shift *after* demodulation and AGC
    if(demod->shift.freq != 0){
      pthread_mutex_lock(&demod->shift.mutex);
      for(int n=0; n < filter->olen; n++){
	filter->output.c[n] *= step_osc(&demod->shift);
      }
      pthread_mutex_unlock(&demod->shift.mutex);
    }
    
    if(demod->output.channels == 1) {
      // Send only I channel as mono
      float samples[filter->olen];
      for(int n=0; n<filter->olen; n++)
	samples[n] = crealf(filter->output.c[n]);
      send_mono_output(demod,samples,filter->olen);
    } else {
      // I on left, Q on right
      send_stereo_output(demod,(float *)filter->output.c,filter->olen);
    }
    // Total baseband power (I+Q), scaled to each sample
    demod->bb_power = (signal + noise) / (2*filter->olen);
    // PLL loop SNR, if used
    if(noise != 0 && demod->pll){
      demod->snr = (signal / noise) - 1; // S/N as power ratio; meaningful only in coherent modes
      if(demod->snr < 0)
	demod->snr = 0; // Clamp to 0 so it'll show as -Inf dB
    } else
      demod->snr = NAN;

  } // terminate
  if(fftinbuf)
    fftwf_free(fftinbuf);
  if(fftoutbuf)
    fftwf_free(fftoutbuf);  
  if(fft_plan)
    fftwf_destroy_plan(fft_plan);
  if(filter)
    delete_filter_output(filter);
  demod->filter.out = NULL;
  pthread_exit(NULL);
}
