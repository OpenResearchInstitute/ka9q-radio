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
  pthread_setname("am");
  struct demod * const demod = arg;
  demod->foffset = NAN; // not used
  demod->pdeviation = NAN;

  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,COMPLEX);
  demod->filter = filter;
  set_filter(demod,demod->low,demod->high);

  while(fillbuf(demod->corr_iq_read_fd,filter->input.c,filter->ilen*sizeof(*filter->input.c)) > 0){
    spindown(demod,filter->input.c,filter->ilen); // 2nd LO
    execute_filter(filter);

    float average = 0;
    float audio[filter->olen];
    
    int n;
    for(n=0; n < filter->olen; n++)
      average += audio[n] = cabs(filter->output.c[n]);
    average /= filter->olen;
    demod->amplitude = average;
    float const snn = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio
    demod->snr = (snn*snn) -1; // S/N as power ratio
    
    // AM AGC is carrier-driven
    //    demod->gain = Headroom / average;
    demod->gain = 0.5/average;
    for(n=0; n<filter->olen; n++)
      audio[n] = (audio[n] - average) * demod->gain;
    
    send_mono_audio(demod->audio,audio,n);
  }
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
