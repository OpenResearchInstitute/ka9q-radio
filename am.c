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
  set_filter(filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);

  while(!demod->terminate){
    fillbuf(demod,filter->input.c,filter->ilen);
    demod->second_LO_phasor = spindown(demod,filter->input.c); // 2nd LO
    demod->if_power = cpower(filter->input.c,filter->ilen);
    execute_filter(filter);
    demod->bb_power = cpower(filter->output.c,filter->olen);
    demod->n0 += .01 * (compute_n0(demod) - demod->n0);

    // Envelope detection
    float average = 0;
    float audio[filter->olen];
    int n;
    for(n=0; n < filter->olen; n++)
      average += audio[n] = cabs(filter->output.c[n]);
    average /= filter->olen;
    
    // AM AGC is carrier-driven
    //    demod->gain = demod->headroom / average;
    demod->gain = 0.5/average;
    // Remove carrier component
    for(n=0; n<filter->olen; n++)
      audio[n] -= average;
    
    send_mono_audio(demod->audio,audio,n,demod->gain);
  }
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
