#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
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

void *demod_iq(void *arg){
  assert(arg != NULL);
  pthread_setname("iq");
  struct demod * const demod = arg;
  demod->foffset = 0; // not used
  demod->pdeviation = NAN;
  const int hangmax = hangtime * (demod->samprate/demod->L); // 1.1 second hang before gain increase
  const float agcratio = dB2voltage(recovery_rate * ((float)demod->L/demod->samprate)); // 6 dB/sec
  int hangcount = 0;

  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,
					       (demod->flags & CONJ) ? CROSS_CONJ : COMPLEX);
  demod->filter = filter;
  set_filter(filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);
  demod->gain = dB2voltage(70.); // Starting point

  while(!demod->terminate){
    fillbuf(demod,filter->input.c,filter->ilen);
    demod->second_LO_phasor = spindown(demod,filter->input.c); // 2nd LO
    demod->if_power = cpower(filter->input.c,filter->ilen);
    execute_filter(filter);
    demod->bb_power = cpower(filter->output.c,filter->olen);
    if(isnan(demod->n0))
      demod->n0 = compute_n0(demod);
    else
      demod->n0 += .01 * (compute_n0(demod) - demod->n0);
    
    // Frequency shift
    if(demod->shift != 0){
      for(int n=0; n < filter->olen; n++){
	filter->output.c[n] *= demod->shift_phasor;
	demod->shift_phasor *= demod->shift_phasor_step;
      }
      demod->shift_phasor /= cabs(demod->shift_phasor);
    }

    // Automatic gain control
    // Find average amplitude for AGC
    float amplitude = sqrtf(demod->bb_power);

    if(demod->gain * amplitude > demod->headroom){ // Target to about -10 dBFS
    // New signal peak: decrease gain and inhibit re-increase for a while
      demod->gain = demod->headroom / amplitude;
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
    send_stereo_audio(demod->audio,filter->output.c,filter->olen,demod->gain);
  }
  delete_filter(demod->filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
