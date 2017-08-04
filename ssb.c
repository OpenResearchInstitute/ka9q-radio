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

static const float Hangtime = 1.1;    // Hang for 1.1 seconds after new peak
static const float Recovery_rate = 10; // Recover gain at this many db/sec after hang finishes


void *demod_ssb(void *arg){
  assert(arg != NULL);
  pthread_setname("ssb");
  struct demod * const demod = arg;
  demod->foffset = NAN; // not used
  demod->pdeviation = NAN;
  demod->gain = dB2voltage(70.); // 70 dB starting point, will adjust with ssb_agc
  int const hangmax = Hangtime * (demod->samprate/demod->L); // 1.1 second hang before gain increase
  float const agcratio = dB2voltage(Recovery_rate * ((float)demod->L/demod->samprate));
  int hangcount = hangmax;

  // Set up pre-demodulation filter
  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,REAL);
  demod->filter = filter;
  set_filter(demod,demod->low,demod->high);

#if 0
  // Experimental notch filter
  struct notchfilter *nf = notch_create(48000./demod->samprate,0.001);
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
    spindown(demod,filter->input.c,filter->ilen); // 2nd LO
    execute_filter(filter);
    // Automatic gain control
    demod->amplitude = amplitude(filter->output.r,filter->olen);
    float const snn = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio
    demod->snr = (snn*snn) -1; // S/N as power ratio
    if(demod->gain * demod->amplitude > demod->headroom){ // Target to about -10 dBFS
      // New signal peak: decrease gain and inhibit re-increase for a while
      demod->gain = demod->headroom / demod->amplitude;
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
      filter->output.r[n] *= demod->gain;

    send_mono_audio(demod->audio,filter->output.r,n);
  }
  delete_filter(demod->filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
