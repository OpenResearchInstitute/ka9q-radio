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

void *ssb_cleanup(void *arg){
  struct demod *demod = arg;
  delete_filter(demod->filter);
  demod->filter = NULL;
  if(Audio.handle){
    snd_pcm_drop(Audio.handle);
    snd_pcm_close(Audio.handle);
    Audio.handle = NULL;
  }
  return NULL;
}


void *demod_ssb(void *arg){
  int n;
  int N;
  const float gain = M_SQRT1_2;  // unity gain
  int low,high;
  struct demod *demod = arg;

  N = demod->L + demod->M - 1;
  enum mode mode = demod->mode;

  demod->foffset = NAN; // not used
  demod->pdeviation = NAN;

  low = N*Modes[mode].low/demod->samprate;
  high = N*Modes[mode].high/demod->samprate;
  if(low > high){
    int t;
    t = low;
    low = high;
    high = t;
  }
  
  demod->gain = dB2voltage(70.); // 70 dB starting point, will adjust with ssb_agc
  int hangmax = 1.1 * (demod->samprate/demod->L); // 1.1 second hang before gain increase
  float agcratio = dB2voltage(6 * ((float)demod->L/demod->samprate)); // 6 dB/sec
  int hangtime = hangmax;



  // Set up pre-demodulation filter
  complex float *response = (complex float *)fftwf_alloc_complex(N);
  // posix_memalign((void **)&response,16,N*sizeof(complex float));
  memset(response,0,N*sizeof(*response));
  for(n=low; n <= high; n++)
    response[(n+N)%N] = gain;
  
  window_filter(demod->L,demod->M,response,Kaiser_beta);
  demod->decimate = demod->samprate / Audio.samprate;
  
  demod->filter = create_filter(demod->L,demod->M,response,demod->decimate,REAL);
  audio_change_parms(Audio.samprate,2,demod->filter->blocksize_out);  

  pthread_cleanup_push(ssb_cleanup,demod);

  while(1){
    fillbuf(demod->data_sock,(char *)demod->filter->input,
	    demod->filter->blocksize_in*sizeof(complex float));

    spindown(demod,demod->filter->input,demod->filter->blocksize_in); // 2nd LO
    execute_filter(demod->filter);
    // Automatic gain control
    demod->amplitude = amplitude(demod->filter->output.r,demod->filter->blocksize_out);
    float snn = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio
    demod->snr = (snn*snn) -1; // S/N as power ratio
    if(demod->gain * demod->amplitude > Headroom){ // Target to about -10 dBFS
      // New signal peak: decrease gain and inhibit re-increase for a while
      demod->gain = Headroom / demod->amplitude;
      hangtime = hangmax;
    } else {
      // Not a new peak, but the AGC is still hanging at the last peak
      if(hangtime !=0){
	hangtime--;
      } else {
	// OK to increase gain; should enforce a limit
	demod->gain *= agcratio;
      }
    }
    put_mono_audio(demod->filter->output.r,demod->filter->blocksize_out,demod->gain);
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}
