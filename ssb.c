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


void *demod_ssb(void *arg){
  assert(arg != NULL);
  pthread_setname("ssb");
  struct demod * const demod = arg;
  demod->foffset = NAN; // not used
  demod->pdeviation = NAN;
  demod->gain = dB2voltage(70.); // 70 dB starting point, will adjust with ssb_agc
  demod->hangtime = 1.1;    // Hang for 1.1 seconds after new peak
  demod->recovery_rate = 10; // Recover gain at this many db/sec after hang finishes

  int const hangmax = demod->hangtime * (demod->samprate/demod->L); // 1.1 second hang before gain increase
  float const agcratio = dB2voltage(demod->recovery_rate * ((float)demod->L/demod->samprate));
  int hangcount = hangmax;

  // Set up pre-demodulation filter
  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,REAL);
  demod->filter = filter;
  set_filter(filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);

#if 0
  // Experimental notch filter
  struct notchfilter * const nf = notch_create(48000./demod->samprate,0.001);
#endif

  while(!demod->terminate){
    fillbuf(demod,filter->input.c,filter->ilen);
#if 0
    // experimental notch
    {
      int i;
      for(i=0;i<filter->ilen;i++)
	filter->input.c[i] = notch(nf,filter->input.c[i]);
    }
#endif
    demod->second_LO_phasor = spindown(demod,filter->input.c); // 2nd LO
    demod->if_power = cpower(filter->input.c,filter->ilen);
    execute_filter(filter);
    demod->bb_power = rpower(filter->output.r,filter->olen);
    demod->n0 += .01 * (compute_n0(demod) - demod->n0);

    // Automatic gain control
    float const amplitude = sqrtf(demod->bb_power);
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
    send_mono_audio(demod->audio,filter->output.r,filter->olen,demod->gain);
  }
  delete_filter(demod->filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
