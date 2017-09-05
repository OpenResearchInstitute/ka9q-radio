// $Id: dsb.c,v 1.12 2017/09/04 00:37:15 karn Exp karn $: DSB-AM / BPSK

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
  float lastblock_angle = NAN;

  demod->gain = dB2voltage(70.);

  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,COMPLEX);
  demod->filter = filter;
  set_filter(filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);

  // Kaiser window and FFT used for coarse frequency acquision
  complex float * const fftbuf = fftwf_alloc_complex(filter->olen);
  fftwf_plan fft_plan = fftwf_plan_dft_1d(filter->olen,fftbuf,fftbuf,FFTW_FORWARD,FFTW_ESTIMATE);
  float kaiser_window[filter->olen];
  make_kaiser(kaiser_window,filter->olen,demod->kaiser_beta);

  while(!demod->terminate){
    // New samples
    complex float if_samples[filter->ilen];
    fillbuf(demod,if_samples,filter->ilen);

    int tries;
    complex double LO_phasor;
    for(tries=0;tries<10;tries++){

      // Tentatively apply 2nd LO
      LO_phasor = spindown(demod,if_samples);
      execute_filter_nocopy(filter); // Stateless execution (don't commit state)
      
      // Perform FFT on squares to ensure peak is in DC bin; extract phase
      int n;
      for(n=0; n < filter->olen; n++){
	complex float const s = filter->output.c[n];
	fftbuf[n] = s * s * kaiser_window[n];
      }
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
      
      if(maxbin == 0)
	break;
      // Coarse retune to correct bin and retry. Each bin is one full cycle per input buffer
      double const delta_f = maxbin / (2.0 * filter->ilen); // cycles per input sample, with /2 for squaring
      set_offset(demod,demod->demod_offset + delta_f); // Adjusts second LO for new offset
      lastblock_angle = NAN; // Don't do fine tuning on next iteration
    }
    // Do this only when we're done iterating
    demod->second_LO_phasor = LO_phasor; // Commit LO
    commit_filter(filter);
    demod->bb_power = cpower(filter->output.c,filter->olen);
    float n0 = compute_n0(demod);
    demod->n0 += .01 * (n0 - demod->n0);

    float const angle = cargf(fftbuf[0])/2; // DC carrier phase  in current block
    if(!isnan(lastblock_angle)){ // Only if last block wasn't a coarse retune
      // Perform fine frequency adjustment
      // How much the phase changed from the last block gives us
      // the frequency offset
      double pdiff = angle_mod(angle - lastblock_angle);
      
      demod->cphase = pdiff; // Radians per block
      pdiff *= demod->samprate / (2 * M_PI * filter->ilen); // cycles per second
      set_offset(demod,demod->demod_offset + 0.25 * pdiff); // Ad hoc constant!!
    }
    lastblock_angle = angle; // Save for comparison with next block
    complex float const phase = csincosf(-angle);

    // Rotate signal onto I axis, compute signal (I) and noise (Q) levels
    float amplitude = 0;
    float noise = 0;
    int n;
    for(n=0; n<filter->olen; n++){
      complex float const rsamp = filter->output.c[n] *= phase;
      amplitude += crealf(rsamp) * crealf(rsamp);
      noise += cimagf(rsamp) * cimagf(rsamp);
    }
    // signal+noise and noise powers
    amplitude /= filter->olen;
    noise /= filter->olen;
    demod->snr = (amplitude / noise) - 1; // S/N as power ratio

    float totampl = sqrtf(amplitude + noise);

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
    for(n=0; n<filter->olen; n++)
      filter->output.c[n] *= demod->gain;
    send_stereo_audio(demod->audio,filter->output.c,n);
  }
  fftwf_free(fftbuf);
  fftwf_destroy_plan(fft_plan);
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
