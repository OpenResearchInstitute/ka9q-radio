#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#undef I
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fftw3.h>

#include "dsp.h"
#include "filter.h"
#include "radio.h"
#include "fm.h"
#include "audio.h"

void *iq_cleanup(void *arg){
  if(Demod.filter != NULL){
    delete_filter(Demod.filter);
    Demod.response = NULL;
    Demod.filter = NULL;
  }
  return NULL;
}


void demod_iq(void *arg){
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
  
  Demod.hangmax = 1.1 * (Demod.samprate/Demod.L); // 1.1 second hang before gain increase
  Demod.agcratio = dB2voltage(6 * ((float)Demod.L/Demod.samprate)); // 6 dB/sec
  Demod.hangtime = 0;

  // Adjust for unity gain
  if(mode == ISB)
    gain = M_SQRT1_2;
  else
    gain = 1.0;

  // Set up pre-demodulation filter
  Demod.response = (complex float *)fftwf_alloc_complex(N);
  // posix_memalign((void **)&Demod.response,16,N*sizeof(complex float));
  memset(Demod.response,0,N*sizeof(*Demod.response));
  for(n=low; n <= high; n++)
    Demod.response[(n+N)%N] = gain;
  
  window_filter(Demod.L,Demod.M,Demod.response,Kaiser_beta);
  Demod.decimate = Demod.samprate / Audio.samprate;
  
  Demod.filter = create_filter(Demod.L,Demod.M,Demod.response,Demod.decimate,
			       mode == ISB ? CROSS_CONJ : COMPLEX);
  Demod.gain = dB2voltage(70.); // Starting point
  //  audio_change_parms(Demod.samprate/Demod.decimate,Modes[mode].channels,Demod.L);
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

    ssb_agc();
    put_stereo_audio(Demod.filter->output.c,Demod.filter->blocksize_out,Demod.gain);
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}
