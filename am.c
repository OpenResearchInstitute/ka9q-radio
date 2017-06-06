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

void am_cleanup(void *arg){
  struct demod *demod = arg;
  delete_filter(demod->filter);
  demod->filter = NULL;
}


void *demod_am(void *arg){
  int n;
  struct demod *demod = arg;

  demod->foffset = NAN; // not used
  demod->pdeviation = NAN;

  demod->decimate = demod->samprate / Audio.samprate;
  demod->filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX);
  set_filter(demod,demod->low,demod->high);
  pthread_cleanup_push(am_cleanup,demod);

  while(1){
    fillbuf(demod->input,(char *)demod->filter->input,
	    demod->filter->blocksize_in*sizeof(complex float));
    spindown(demod,demod->filter->input,demod->filter->blocksize_in); // 2nd LO
    execute_filter(demod->filter);

    float average;
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
      audio[n] = (audio[n] - average) * demod->gain;
    
    write(demod->output,audio,sizeof(audio));
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}
