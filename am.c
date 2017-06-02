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

void *am_cleanup(void *arg){
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


void *demod_am(void *arg){
  int n;
  int N;
  const float gain = 1.0; // Unity
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
  // Set up pre-demodulation filter
  complex float *response = (complex float *)fftwf_alloc_complex(N);
  // posix_memalign((void **)&response,16,N*sizeof(complex float));
  memset(response,0,N*sizeof(*response));
  for(n=low; n <= high; n++)
    response[(n+N)%N] = gain;
  
  window_filter(demod->L,demod->M,response,Kaiser_beta);
  demod->decimate = demod->samprate / Audio.samprate;
  demod->filter = create_filter(demod->L,demod->M,response,demod->decimate,COMPLEX);
  audio_change_parms(Audio.samprate,2,demod->filter->blocksize_out);  

  pthread_cleanup_push(am_cleanup,demod);

  while(1){
    fillbuf(demod->data_sock,(char *)demod->filter->input,
	    demod->filter->blocksize_in*sizeof(complex float));
    spindown(demod,demod->filter->input,demod->filter->blocksize_in); // 2nd LO
    execute_filter(demod->filter);

    float average;
    int n;
    float audio[demod->filter->blocksize_out];
    
    average = 0;
    for(n=0; n < demod->filter->blocksize_out; n++)
      average += audio[n] = cabs(demod->filter->output.c[n]);
    average /= demod->filter->blocksize_out;
    demod->amplitude = average;
    float snn = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio
    demod->snr = (snn*snn) -1; // S/N as power ratio
    
    // AM AGC is carrier-driven
    demod->gain = Headroom / average;
    for(n=0; n<demod->filter->blocksize_out; n++)
      audio[n] -= average; // Subtract carrier to remove DC
    
    put_mono_audio(audio,demod->filter->blocksize_out,demod->gain); // we do our own
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}
