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
  demod->foffset = 0; // not used
  demod->pdeviation = NAN;

  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,COMPLEX);
  demod->filter = filter;
  set_filter(filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);

  float average_average = 0;

  // AGC
  int hangcount = 0;
  float samptime = demod->decimate / demod->samprate;
  static float const hangtime = 1.1;      // AGC Hang for 1.1 seconds after new peak
  static float const recovery_rate = 6;   // AGC recovery rate 6 db/sec after hang finishes
  static float const attack_rate = -30;    // Attack 30 dB/sec

  float const recovery_factor = dB2voltage(recovery_rate * samptime);
  float const attack_factor = dB2voltage(attack_rate * samptime);
  int const hangmax = hangtime / samptime; // 1.1 second hang before gain increase

  while(!demod->terminate){
    fillbuf(demod,filter->input.c,filter->ilen);
    demod->second_LO_phasor = spindown(demod,filter->input.c); // 2nd LO
    demod->if_power = cpower(filter->input.c,filter->ilen);
    execute_filter(filter);
    //    demod->bb_power = cpower(filter->output.c,filter->olen); // do this below to save time
    if(isnan(demod->n0))
      demod->n0 = compute_n0(demod);
    else
      demod->n0 += .01 * (compute_n0(demod) - demod->n0);

    // Envelope detection & AGC
    float audio[filter->olen];
    demod->bb_power = 0;
    for(int n=0; n < filter->olen; n++){
      float const t = cnrmf(filter->output.c[n]);
      float const mag = sqrtf(t);
      
      demod->bb_power += t;
      audio[n] = mag * demod->gain;
      if(fabsf(audio[n]) > demod->headroom){
	demod->gain *= attack_factor;
	audio[n] = mag * demod->gain;
	hangcount = hangmax;
      } else if(hangcount != 0){
	hangcount--;
      } else {
	demod->gain *= recovery_factor;
      }
    }
    demod->bb_power /= filter->olen;
    send_mono_audio(demod->audio,audio,filter->olen,1);

  }
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
