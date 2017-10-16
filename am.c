// $Id: am.c,v 1.26 2017/10/10 12:19:32 karn Exp karn $

// New AM demodulator, stripped from linear.c
// Oct 9 2017 Phil Karn, KA9Q

#define _GNU_SOURCE 1
#include <complex.h>
#include <math.h>
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


void *demod_am(void *arg){
  pthread_setname("am");
  assert(arg != NULL);
  struct demod * const demod = arg;

  // Set derived (and other) constants
  float const samptime = demod->decimate / demod->samprate;  // Time between (decimated) samples

  // AGC
  int hangcount = 0;
  float const recovery_factor = dB2voltage(demod->recovery_rate * samptime); // AGC ramp-up rate/sample
  float const attack_factor = dB2voltage(demod->attack_rate * samptime);      // AGC ramp-down rate/sample
  int const hangmax = demod->hangtime / samptime; // samples before AGC increase
  demod->gain = dB2voltage(40.); 

  // DC removal from envelope-detected AM and coherent AM
  float DC_filter = 0;
  float const DC_filter_coeff = .0001;

  demod->flags |= MONO; // Implies mono

  demod->snr = -INFINITY;

  // Detection filter
  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,
					       (demod->flags & CONJ) ? CROSS_CONJ : COMPLEX);
  demod->filter = filter;
  set_filter(filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);

  while(!demod->terminate){
    // New samples
    fillbuf(demod,filter->input.c,filter->ilen);
    spindown(demod,filter->input.c);
    demod->if_power = cpower(filter->input.c,filter->ilen);
    execute_filter(filter);
    if(!isnan(demod->n0))
      demod->n0 += .001 * (compute_n0(demod) - demod->n0);
    else
      demod->n0 = compute_n0(demod); // Happens at startup

    // Demodulation
    float signal = 0;
    float noise = 0;
    // Envelope detected AM
    float audio[filter->olen];
    for(int n=0; n<filter->olen; n++){
      float const sampsq = cnrmf(filter->output.c[n]);
      signal += sampsq;
      float samp = sqrtf(sampsq);
      
      // Remove carrier DC, use for AGC
      // DC_filter will always be positive since sqrtf() is positive
      DC_filter += DC_filter_coeff * (samp - DC_filter);
      
      if(isnan(demod->gain)){
	demod->gain = demod->headroom / DC_filter;
      } else if(demod->gain * DC_filter > demod->headroom){
	demod->gain *= attack_factor;
	hangcount = hangmax;
      } else if(hangcount != 0){
	hangcount--;
      } else {
	demod->gain *= recovery_factor;
      }
      audio[n] = (samp - DC_filter) * demod->gain;
    }
    send_mono_audio(demod->audio,audio,filter->olen);
    demod->bb_power = (signal + noise) / filter->olen;
  } // terminate
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
