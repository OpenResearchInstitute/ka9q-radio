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

void *iq_cleanup(void *arg){
  struct demod *demod = arg;
  if(demod->filter != NULL){
    delete_filter(demod->filter);
    demod->filter = NULL;
  }
  return NULL;
}


void *demod_iq(void *arg){
  int n;
  int N;
  float gain;
  int low,high;
  struct demod *demod = arg;

  N = demod->L + demod->M - 1;
  enum mode mode = demod->mode;

  low = N*Modes[mode].low/demod->samprate;
  high = N*Modes[mode].high/demod->samprate;
  if(low > high){
    int t;
    t = low;
    low = high;
    high = t;
  }
  
  int hangmax = 1.1 * (demod->samprate/demod->L); // 1.1 second hang before gain increase
  float agcratio = dB2voltage(6 * ((float)demod->L/demod->samprate)); // 6 dB/sec
  int hangtime = 0;

  // Adjust for unity gain
  gain = (mode == ISB) ? M_SQRT1_2 : 1.0;

  // Set up pre-demodulation filter
  complex float *response = (complex float *)fftwf_alloc_complex(N);
  // posix_memalign((void **)&response,16,N*sizeof(complex float));
  memset(response,0,N*sizeof(*response));
  for(n=low; n <= high; n++)
    response[(n+N)%N] = gain;
  
  window_filter(demod->L,demod->M,response,Kaiser_beta);
  demod->decimate = demod->samprate / Audio.samprate;
  
  demod->filter = create_filter(demod->L,demod->M,response,demod->decimate,
			       mode == ISB ? CROSS_CONJ : COMPLEX);
  demod->gain = dB2voltage(70.); // Starting point
  audio_change_parms(Audio.samprate,2,demod->filter->blocksize_out);  

  pthread_cleanup_push(iq_cleanup,demod);

  while(1){
    complex float *buffer;
    read(demod->data_sock,&buffer,sizeof(buffer));
    
    memcpy(demod->filter->input,buffer,demod->filter->blocksize_in*sizeof(*buffer));

    spindown(demod,demod->filter->input,demod->filter->blocksize_in); // 2nd LO

    int i;
    i = execute_filter(demod->filter);
    assert(i == 0);
    // Automatic gain control
    // Find average amplitude for AGC
    demod->amplitude = camplitude(demod->filter->output.c,demod->filter->blocksize_out);
    float snn = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio
    demod->snr = (snn*snn) -1; // S/N as power ratio

    if(demod->gain * demod->amplitude > Headroom){ // Target to about -10 dBFS
    // New signal peak: decrease gain and inhibit re-increase for a while
      demod->gain = Headroom / demod->amplitude;
      hangtime = hangmax;
    } else {
      // Not a new peak, but the AGC is still hanging at the last peak
      if(hangtime !=0){
	hangtime--;
      } else {
	// OK to increase gain; should enforce a limit
	demod->gain *= agcratio;
      }
    }
    put_stereo_audio(demod->filter->output.c,demod->filter->blocksize_out,demod->gain);
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}
