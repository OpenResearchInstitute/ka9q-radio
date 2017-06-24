// DSB-AM / BPSK

#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <math.h>
#include <complex.h>
#undef I
#include <fftw3.h>

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


void *demod_dsb(void *arg){
  struct demod *demod = arg;
  int hangcount = 0;
  const float agcratio = dB2voltage(recovery_rate * ((float)demod->L/demod->samprate)); // 6 dB/sec
  const int hangmax = hangtime * (demod->samprate/demod->L); // 1.1 second hang before gain increase
  complex float pll = 1;

  pthread_setname_np(pthread_self(),"dsb");
  demod->foffset = NAN; // not used
  demod->pdeviation = NAN;
  demod->gain = dB2voltage(70.);

  pthread_cleanup_push(dsb_cleanup,demod);
  demod->filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX);
  set_filter(demod,demod->low,demod->high);

  while(1){
    fillbuf(demod->input,(char *)demod->filter->input,
	    demod->filter->blocksize_in*sizeof(complex float));
    spindown(demod,demod->filter->input,demod->filter->blocksize_in); // 2nd LO
    execute_filter(demod->filter);
    
    // Costas feedback
    float phase = 0;
    float amplitude=0,noise=0;
    int n;

    for(n=0; n < demod->filter->blocksize_out; n++){
      complex float rsamp = demod->filter->output.c[n] * pll;
      phase += crealf(rsamp) * cimagf(rsamp);
      amplitude += creal(rsamp) * creal(rsamp);
      noise += cimag(rsamp) * cimag(rsamp);
    }
    // RMS signal+noise and noise amplitudes
    amplitude /= demod->filter->blocksize_out;
    noise /= demod->filter->blocksize_out;
    demod->amplitude = sqrtf(amplitude); // RMS amplitude of I channel
    demod->noise = sqrtf(noise);         // RMS amplitude of Q channel
    demod->snr = demod->amplitude / demod->noise; // (S+N)/N ratio

    // Complete PLL feedback loop - adjust LO
    // Positive phase means signal is positive frequency, so increase LO frequency
    // to catch up with it
    phase = phase / (noise + amplitude);
    double freqerror = 0.00001 * phase * M_1_2PI * demod->samprate/demod->filter->blocksize_in;
    set_second_LO(demod,freqerror + demod->second_LO,0);

    if(demod->gain * demod->amplitude > Headroom){ // Target to about -10 dBFS
      // New signal peak: decrease gain and inhibit re-increase for a while
      demod->gain = Headroom / demod->amplitude;
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
    complex float audio[demod->filter->blocksize_out];
    for(n=0;n<demod->filter->blocksize_out;n++)
      audio[n] = demod->filter->output.c[n] * demod->gain;
    send_stereo_audio(audio,n);
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}
