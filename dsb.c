// $Id: dsb.c,v 1.14 2017/09/07 02:47:08 karn Exp karn $: DSB-AM / BPSK

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

static float const hangtime = 1.1;    // Hang for 1.1 seconds after new peak
static float const recovery_rate = 6; // Recover gain at 6 db/sec after hang finishes


void *demod_dsb(void *arg){
  pthread_setname("dsb");
  assert(arg != NULL);

  struct demod * const demod = arg;
  int hangcount = 0;
  float const agcratio = dB2voltage(recovery_rate * ((float)demod->L/demod->samprate)); // 6 dB/sec
  int const hangmax = hangtime * (demod->samprate/demod->L); // 1.1 second hang before gain increase

  demod->gain = dB2voltage(70.);

  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,COMPLEX);
  demod->filter = filter;
  set_filter(filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);

  // Kaiser window and FFT used for coarse frequency acquision
  complex float * const fftbuf = fftwf_alloc_complex(filter->olen);
  fftwf_plan fft_plan = fftwf_plan_dft_1d(filter->olen,fftbuf,fftbuf,FFTW_FORWARD,FFTW_ESTIMATE);
  float kaiser_window[filter->olen];
  make_kaiser(kaiser_window,filter->olen,demod->kaiser_beta);


  complex double offset_phasor = 1; // offset LO
  float integrator = 0; // 2nd order loop integrator
  complex double offset_step = 1;
  const double samptime = demod->decimate / demod->samprate;

  // Proportionality constant for offset oscillator in radians per decimated (output) sample per hertz
  const double phase_scale = 2 * M_PI * samptime;
  // Second-order loop (see Gardner)
  const float vcogain = 2*M_PI; // 1 Hz = 2pi radians/sec per "volt"
  const float pdgain = 1;       // "volts" per radian (unity)
  const float natfreq = 1 * (2*M_PI);   // natural frequency 1 Hz = 2*pi rad/s
  const float tau1 = vcogain * pdgain / (natfreq*natfreq); // 1 / 2pi
  const float integrator_gain = 1 / tau1; // 2pi
  const float damping = M_SQRT1_2;
  const float tau2 = 2 * damping / natfreq; // sqrt(2) / 2pi = 1/ (pi*sqrt(2))
  const float prop_gain = tau2 / tau1;    // sqrt(2)/2

  while(!demod->terminate){
    // New samples
    complex float if_samples[filter->ilen];
    fillbuf(demod,if_samples,filter->ilen);

    int tries;
    complex double LO_phasor = 1;
    for(tries=0;tries<2;tries++){

      // Tentatively apply 2nd LO
      LO_phasor = spindown(demod,if_samples);
      execute_filter_nocopy(filter); // Stateless execution (don't commit state)
      
#if !defined(COARSE)
      break;
#endif

      // Perform FFT to ensure peak is in DC bin; extract phase
      int n;
#if SQUARE
      for(n=0; n < filter->olen; n++){
	complex float const s = filter->output.c[n];
	fftbuf[n] = s * s * kaiser_window[n];
      }
#else
      for(n=0; n < filter->olen; n++)
	fftbuf[n] = filter->output.c[n] * kaiser_window[n];
#endif	
      fftwf_execute(fft_plan);
      int maxbin = 0;
      float maxenergy = 0;
      for(n=0;n<filter->olen;n++){
	float const e = cnrmf(fftbuf[n]);
	if(e > maxenergy){
	  maxenergy = e;
	  maxbin = n;
	}
      }
      if(maxbin >= filter->olen/2)
	maxbin -= filter->olen; // negative frequency
      
      if(maxbin == 0) // Locked onto right bin
	break;
      // Coarse retune to correct bin and retry. Each bin is two full cycles of carrier due to squaring
      double delta_f = demod->samprate * maxbin / filter->ilen; // cycles per second
#if SQUARE
      delta_f /= 2;
#endif
      double new_offset = demod->demod_offset + delta_f;
      if(new_offset > 200)
	new_offset = 200;
      if(new_offset < -200)
	new_offset = -200;

      set_offset(demod,new_offset); // Adjusts second LO for new offset
    }



    // Lag-lead (integral plus proportional)

    // Apply offset, determine phase
    int n;
    complex float accum = 0;
    double offset = demod->demod_offset; // Grab value that might have been manually changed
    for(n=0;n<filter->olen;n++){
      filter->output.c[n] *= offset_phasor;
      accum += filter->output.c[n];
      double carrier_phase = carg(filter->output.c[n]);
      integrator += carrier_phase * samptime;
      float feedback = integrator * integrator_gain + prop_gain * carrier_phase;
      offset = feedback > 100 ? 100 : feedback < -100 ? -100 : feedback;
      offset_step = csincos(-phase_scale * offset);
      offset_phasor *= offset_step;
    }
    demod->cphase = cargf(accum);
    offset_phasor /= cabs(offset_phasor);
    demod->demod_offset = offset;

    // We're finally done iterating on frequency
    demod->second_LO_phasor = LO_phasor; // Commit LO
    commit_filter(filter);
    demod->if_power = cpower(filter->input.c,filter->ilen);
    demod->bb_power = cpower(filter->output.c,filter->olen);
    demod->n0 += .01 * (compute_n0(demod) - demod->n0);
#if 0
    // Rotate onto I axis
    {
      assert(!isnan(demod->cphase));
      complex float const phase = csincos(-demod->cphase);
      assert(!isnan(crealf(phase)) && !isnan(cimagf(phase)));
      int n;
      for(n=0;n<filter->olen;n++)
	filter->output.c[n] *= phase;
    }
#endif
    // compute signal (I) and noise (Q) levels, total level and SNR
    float totampl;
    {
      complex float const p = cpowers(filter->output.c,filter->olen);
      float const amplitude = crealf(p);
      float const noise = cimagf(p);
      assert(!isnan(amplitude) && !isnan(noise));
      // signal+noise and noise powers
      demod->snr = (amplitude / noise) - 1; // S/N as power ratio
      totampl = sqrtf(amplitude + noise);
    }

    // Q is on the right channel, so use both I and Q for gain setting so we don't blast our ears when the phase is wrong
    if(demod->gain * totampl > demod->headroom){ // Target to about -10 dBFS
      // New signal peak: decrease gain and inhibit re-increase for a while
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
  fftwf_free(fftbuf);
  fftwf_destroy_plan(fft_plan);
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
