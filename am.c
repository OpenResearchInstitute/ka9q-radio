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



void *demod_am(void *arg){
  assert(arg != NULL);

  int n;
  struct demod *demod = arg;

  pthread_setname_np(pthread_self(),"am");
  demod->foffset = NAN; // not used
  demod->pdeviation = NAN;

  struct filter * filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX);
  demod->filter = filter;
  set_filter(demod,demod->low,demod->high);

  while(!demod->terminate){
    fillbuf(demod->input,(char *)filter->input,
	    filter->blocksize_in*sizeof(complex float));
    spindown(demod,filter->input,filter->blocksize_in); // 2nd LO
    execute_filter(filter);

    float average;
    float audio[filter->blocksize_out];
    
    average = 0;
    for(n=0; n < filter->blocksize_out; n++)
      average += audio[n] = cabs(filter->output.c[n]);
    average /= filter->blocksize_out;
    demod->amplitude = average;
    float snn = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio
    demod->snr = (snn*snn) -1; // S/N as power ratio
    
    // AM AGC is carrier-driven
    demod->gain = Headroom / average;
    for(n=0; n<filter->blocksize_out; n++)
      audio[n] = (audio[n] - average) * demod->gain;
    
    send_mono_audio(audio,n);
  }
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
