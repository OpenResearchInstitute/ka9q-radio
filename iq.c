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
  if(Demod.filter != NULL){
    delete_filter(Demod.filter);
    Demod.filter = NULL;
  }
  return NULL;
}


void *demod_iq(void *arg){
  int n;
  int N;
  float gain;
  int low,high;

  N = Demod.L + Demod.M - 1;
  enum mode mode = Demod.mode;

  low = N*Modes[mode].low/Demod.samprate;
  high = N*Modes[mode].high/Demod.samprate;
  if(low > high){
    int t;
    t = low;
    low = high;
    high = t;
  }
  
  float hangmax = 1.1 * (Demod.samprate/Demod.L); // 1.1 second hang before gain increase
  float agcratio = dB2voltage(6 * ((float)Demod.L/Demod.samprate)); // 6 dB/sec
  float hangtime = 0;

  // Adjust for unity gain
  if(mode == ISB)
    gain = M_SQRT1_2;
  else
    gain = 1.0;

  // Set up pre-demodulation filter
  complex float *response = (complex float *)fftwf_alloc_complex(N);
  // posix_memalign((void **)&response,16,N*sizeof(complex float));
  memset(response,0,N*sizeof(*response));
  for(n=low; n <= high; n++)
    response[(n+N)%N] = gain;
  
  window_filter(Demod.L,Demod.M,response,Kaiser_beta);
  Demod.decimate = Demod.samprate / Audio.samprate;
  
  Demod.filter = create_filter(Demod.L,Demod.M,response,Demod.decimate,
			       mode == ISB ? CROSS_CONJ : COMPLEX);
  Demod.gain = dB2voltage(70.); // Starting point
  audio_change_parms(Audio.samprate,2,Demod.filter->blocksize_out);  

  pthread_cleanup_push(iq_cleanup,&Demod);

  while(1){
    complex float *buffer;
    read(Demod.data_sock,&buffer,sizeof(buffer));
    
    memcpy(Demod.filter->input,buffer,Demod.filter->blocksize_in*sizeof(*buffer));

    spindown(Demod.filter->input,Demod.filter->blocksize_in); // 2nd LO

    int i;
    i = execute_filter(Demod.filter);
    assert(i == 0);
    // Automatic gain control
    // Find average amplitude for AGC
    Demod.amplitude = camplitude(Demod.filter->output.c,Demod.filter->blocksize_out);
    float snn = Demod.amplitude / Demod.noise; // (S+N)/N amplitude ratio
    Demod.snr = (snn*snn) -1; // S/N as power ratio

    if(Demod.gain * Demod.amplitude > Headroom){ // Target to about -10 dBFS
    // New signal peak: decrease gain and inhibit re-increase for a while
      Demod.gain = Headroom / Demod.amplitude;
      hangtime = hangmax;
    } else {
      // Not a new peak, but the AGC is still hanging at the last peak
      if(hangtime !=0){
	hangtime--;
      } else {
	// OK to increase gain; should enforce a limit
	Demod.gain *= agcratio;
      }
    }
    put_stereo_audio(Demod.filter->output.c,Demod.filter->blocksize_out,Demod.gain);
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}
