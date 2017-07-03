// $Id: dsb.c,v 1.4 2017/07/02 12:02:13 karn Exp karn $: DSB-AM / BPSK

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
  assert(arg != NULL);
  pthread_setname_np(pthread_self(),"dsb");
  struct demod * const demod = arg;
  int hangcount = 0;
  float const agcratio = dB2voltage(recovery_rate * ((float)demod->L/demod->samprate)); // 6 dB/sec
  int const hangmax = hangtime * (demod->samprate/demod->L); // 1.1 second hang before gain increase
  int const mm1 = demod->M - 1;
  float lastblock_phase = NAN;
  int const N = demod->L + mm1;

  demod->gain = dB2voltage(70.);

  struct filter * const filter = create_filter(demod->L,mm1+1,NULL,demod->decimate,COMPLEX);
  demod->filter = filter;
  set_filter(demod,demod->low,demod->high);

  // Kaiser window and FFT used for coarse frequency acquision
  complex float * const fftbuf = fftwf_alloc_complex(filter->olen);
  fftwf_plan fft_plan = fftwf_plan_dft_1d(filter->olen,fftbuf,fftbuf,FFTW_FORWARD,FFTW_ESTIMATE);
  float kaiser_window[filter->olen];
  make_kaiser(kaiser_window,filter->olen,Kaiser_beta);

  // This filter is unconventional because we iterate on the second LO, which means we
  // have to recompute the *entire* filter input buffer every time
  complex float if_samples[N];
  fillbuf(demod->input,if_samples+filter->ilen,mm1 * sizeof(*if_samples)); // Prime the pump
  while(!demod->terminate){
    memmove(if_samples,if_samples+filter->ilen,mm1*sizeof(*if_samples)); // Re-copy the overlap so we can downconvert it again
    fillbuf(demod->input,if_samples+mm1,filter->ilen*sizeof(*if_samples)); // New samples

    complex double LO_phase_step = demod->second_LO_phase_step;
    int tries;
    complex double updated_LO_phase = NAN; // LO phase after M-1 sample, needed for next block if we're successful
    for(tries=0;tries<10;tries++){

      // Apply 2nd LO, filling *entire* filter input buffer
      complex double LO_phase = demod->second_LO_phase;

      int n;
      for(n=0; n < N; n++){
	assert(!isnan(crealf(if_samples[n])) && !isnan(cimagf(if_samples[n])));
	filter->input_buffer[n] = if_samples[n] * LO_phase;
	LO_phase *= LO_phase_step;
	if(n == mm1 - 1)
	  updated_LO_phase = LO_phase; // Starting LO phase for next block if we're successful
      }
      execute_filter_nocopy(filter); // Stateless execution (no overlap)
      
      // Perform FFT on squares to ensure peak is in DC bin; extract phase
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
      LO_phase_step *= csincos(-delta_f * 2 * M_PI); // rad per input sample
      lastblock_phase = NAN; // Don't do fine tuning on next iteration
    }
    float const phase = cargf(fftbuf[0])/2; // DC carrier phase  in current block
    if(!isnan(lastblock_phase)){ // Only if last block wasn't a coarse retune
      // Perform fine frequency adjustment
      // How much the phase changed from the last block gives us
      // the frequency offset
      double pdiff = phase - lastblock_phase;
      if(pdiff > M_PI)
	pdiff -= 2*M_PI;
      else if(pdiff < -M_PI)
	pdiff += 2*M_PI;
      
      demod->cphase = pdiff;
      LO_phase_step *= csincos(-pdiff / filter->ilen); // pdiff radians in one input blk
    }
    lastblock_phase = phase; // Save for comparison with next block
    demod->second_LO_phase = updated_LO_phase /= cabs(updated_LO_phase); // Save renormalized
    demod->second_LO_phase_step = LO_phase_step /= cabs(LO_phase_step);
    demod->second_LO = M_1_2PI * demod->samprate * carg(demod->second_LO_phase_step); // phase step converted to Hz
    
    // Rotate already demodulated signal onto I axis, compute signal (I) and noise (Q) amplitudes
    float amplitude = 0;
    float noise = 0;
    complex float const rot = csincosf(-phase);
    int n;
    for(n=0;n<filter->olen;n++){
      complex float const sample = filter->output.c[n] * rot;
      filter->output.c[n] = sample;
      amplitude += crealf(sample) * crealf(sample);
      noise += cimagf(sample) * cimagf(sample);
    }
    // RMS signal+noise and noise amplitudes
    amplitude /= filter->olen;
    noise /= filter->olen;
    demod->amplitude = sqrtf(amplitude); // RMS amplitude of I channel
    demod->noise = sqrtf(noise);         // RMS amplitude of Q channel
    demod->snr = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio

    // Use both I and Q for gain setting so we don't blast our ears when the phase is wrong
    if(demod->gain * sqrtf(amplitude+noise) > Headroom){ // Target to about -10 dBFS
      // New signal peak: decrease gain and inhibit re-increase for a while
      //      demod->gain = Headroom / demod->amplitude;
      demod->gain = Headroom / sqrtf(amplitude + noise);
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
    complex float audio[filter->olen];
    for(n=0;n<filter->olen;n++)
      audio[n] = filter->output.c[n] * demod->gain;
    send_stereo_audio(audio,n);
  }
  fftwf_free(fftbuf);
  fftwf_destroy_plan(fft_plan);
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
