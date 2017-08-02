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
  demod->foffset = NAN; // not used
  demod->pdeviation = NAN;
  const int hangmax = hangtime * (demod->samprate/demod->L); // 1.1 second hang before gain increase
  const float agcratio = dB2voltage(recovery_rate * ((float)demod->L/demod->samprate)); // 6 dB/sec
  int hangcount = 0;

  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,
			       demod->mode == ISB ? CROSS_CONJ : COMPLEX);
  demod->filter = filter;
  set_filter(demod,demod->low,demod->high);
  demod->gain = dB2voltage(70.); // Starting point

  while(fillbuf(demod->corr_iq_read_fd,filter->input.c,filter->ilen*sizeof(*filter->input.c)) > 0){
    spindown(demod,filter->input.c,filter->ilen); // 2nd LO
    execute_filter(filter);

    // Automatic gain control
    // Find average amplitude for AGC
    demod->amplitude = camplitude(filter->output.c,filter->olen);
    float const snn = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio
    demod->snr = (snn*snn) -1; // S/N as power ratio

    if(demod->gain * demod->amplitude > Headroom){ // Target to about -10 dBFS
    // New signal peak: decrease gain and inhibit re-increase for a while
      demod->gain = Headroom / demod->amplitude;
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
    int n;
    for(n=0;n<filter->olen;n++)
      filter->output.c[n] *= demod->gain;

    send_stereo_audio(demod->audio,filter->output.c,n);
  }
  delete_filter(demod->filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
