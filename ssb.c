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

static const float hangtime = 1.1;    // Hang for 1.1 seconds after new peak
static const float recovery_rate = 6; // Recover gain at 6 db/sec after hang finishes


void ssb_cleanup(void *arg){
  struct demod *demod = arg;
  delete_filter(demod->filter);
  demod->filter = NULL;
}


void *demod_ssb(void *arg){
  int n;
  struct demod *demod = arg;

  demod->foffset = NAN; // not used
  demod->pdeviation = NAN;

  demod->gain = dB2voltage(70.); // 70 dB starting point, will adjust with ssb_agc
  int hangmax = hangtime * (demod->samprate/demod->L); // 1.1 second hang before gain increase
  float agcratio = dB2voltage(recovery_rate * ((float)demod->L/demod->samprate)); // 6 dB/sec
  int hangcount = hangmax;

  // Set up pre-demodulation filter
  demod->filter = create_filter(demod->L,demod->M,NULL,demod->decimate,REAL);
  set_filter(demod,demod->low,demod->high);
  pthread_cleanup_push(ssb_cleanup,demod);

  while(1){
    fillbuf(demod->input,(char *)demod->filter->input,
	    demod->filter->blocksize_in*sizeof(complex float));

    spindown(demod,demod->filter->input,demod->filter->blocksize_in); // 2nd LO
    execute_filter(demod->filter);
    // Automatic gain control
    demod->amplitude = amplitude(demod->filter->output.r,demod->filter->blocksize_out);
    float snn = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio
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
    for(n=0;n<demod->filter->blocksize_out;n++)
      demod->filter->output.r[n] *= demod->gain;

    write(demod->output,demod->filter->output.r,
	  demod->filter->blocksize_out*sizeof(*demod->filter->output.r));
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}
