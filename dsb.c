// DSB-AM / BPSK

#define _GNU_SOURCE 1
#include <complex.h>
#include <math.h>
#include <fftw3.h>
#include <assert.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>


#include "dsp.h"
#include "filter.h"
#include "radio.h"
#include "audio.h"

static const float hangtime = 1.1;    // Hang for 1.1 seconds after new peak
static const float recovery_rate = 6; // Recover gain at 6 db/sec after hang finishes


void dsb_cleanup(void *arg){
  struct demod *demod = arg;
  delete_filter(demod->filter);
  demod->filter = NULL;
}


float DSB_offset;
float DSB_phase;


void *demod_dsb(void *arg){
  struct demod *demod = arg;
  int hangcount = 0;
  const float agcratio = dB2voltage(recovery_rate * ((float)demod->L/demod->samprate)); // 6 dB/sec
  const int hangmax = hangtime * (demod->samprate/demod->L); // 1.1 second hang before gain increase
  const int ilen = demod->L;
  const int olen = demod->L / demod->decimate;
  float lastblock_phase = 0;

  pthread_setname_np(pthread_self(),"dsb");
  demod->foffset = NAN; // not used
  demod->pdeviation = NAN;
  demod->gain = dB2voltage(70.);

  pthread_cleanup_push(dsb_cleanup,demod);
  demod->filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX);
  set_filter(demod,demod->low,demod->high);

  if(demod->second_LO_phase == 0)
    demod->second_LO_phase = 1; // Ensure initialization

  complex float *fftbuf = fftwf_alloc_complex(olen);
  fftwf_plan fft_plan = fftwf_plan_dft_1d(olen,fftbuf,fftbuf,FFTW_FORWARD,FFTW_ESTIMATE);

  float *kaiser_window = malloc(olen * sizeof(float));
  make_kaiser(kaiser_window,olen,3.0);

  while(1){
    complex float if_samples[ilen];
    fillbuf(demod->input,if_samples,sizeof(if_samples));

    complex double LO_phase_step = demod->second_LO_phase_step;
    // Apply 2nd LO
    complex double LO_phase = demod->second_LO_phase;
    int n;
    for(n=0; n < ilen; n++){
      demod->filter->input[n] = if_samples[n] * LO_phase;
      LO_phase *= LO_phase_step;
    }
    execute_filter(demod->filter);

    // Perform FFT on squares to ensure peak is in DC bin; extract phase
    for(n=0; n < olen; n++){
      complex float s = demod->filter->output.c[n];
      fftbuf[n] = s * s * kaiser_window[n];
    }
    fftwf_execute(fft_plan);
    int maxbin = 0;
    float maxenergy = 0;
    for(n=0;n<olen;n++){
      float e = cnrmf(fftbuf[n]);
      if(e > maxenergy){
	maxenergy = e;
	maxbin = n;
      }
    }
    if(maxbin >= olen/2)
      maxbin -= olen; // negative frequency

    float phase;
    if(maxbin != 0){
      // Retune to correct bin. Each bin is one full cycle per input buffer
      double delta_f = maxbin / (double)ilen; // cycles per input sample
      delta_f /= 2; // Undo doubling of frequency from squaring
      LO_phase_step *= csincos(-delta_f * 2 * M_PI); // rad per input sample
      lastblock_phase = NAN;
      phase = 0;
    } else {
      phase = cargf(fftbuf[0])/2; // DC carrier phase

      if(!isnan(lastblock_phase)){
	// Perform fine frequency adjustment
	// How much the phase changed from the last block gives us
	// the frequency offset
	double pdiff = phase - lastblock_phase;
	if(pdiff > M_PI)
	  pdiff -= 2*M_PI;
	else if(pdiff < -M_PI)
	  pdiff += 2*M_PI;
	
	LO_phase_step *= csincos(-(pdiff / ilen)/1.); // pdiff radians in one input blk
      }
      lastblock_phase = phase;
    }
    demod->cphase = phase;
    demod->second_LO_phase = LO_phase /= cabs(LO_phase); // Save renormalized
    demod->second_LO_phase_step = LO_phase_step /= cabs(LO_phase_step);
    demod->second_LO = M_1_2PI * demod->samprate * carg(demod->second_LO_phase_step);

    // Rotate already demodulated symbols, compute signal and noise amplitudes
    float amplitude = 0;
    float noise = 0;
    complex float rot = csincosf(-phase);
    for(n=0;n<olen;n++){
      complex float sample = demod->filter->output.c[n] * rot;
      amplitude += crealf(sample) * crealf(sample);
      noise += cimagf(sample) * cimagf(sample);
      demod->filter->output.c[n] = sample;
    }
    // RMS signal+noise and noise amplitudes
    amplitude /= olen;
    noise /= olen;
    demod->amplitude = sqrtf(amplitude); // RMS amplitude of I channel
    demod->noise = sqrtf(noise);         // RMS amplitude of Q channel
    demod->snr = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio

    //    if(demod->gain * demod->amplitude > Headroom){ // Target to about -10 dBFS
    if(demod->gain * sqrtf(amplitude+noise) > Headroom){ // Target to about -10 dBFS
      // New signal peak: decrease gain and inhibit re-increase for a while
      //      demod->gain = Headroom / demod->amplitude;
      demod->gain = Headroom / sqrtf(amplitude + noise);
      hangcount = hangmax;
    } else {
      // Not a new peak, but the AGC is still hanging at the last peak
      if(hangcount !=0){
	hangcount--;
      } else {
	// OK to increase gain; should enforce a limit
	demod->gain *= agcratio;
      }
    }
    complex float audio[olen];
    for(n=0;n<olen;n++)
      audio[n] = demod->filter->output.c[n] * demod->gain;
    send_stereo_audio(audio,n);
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}
