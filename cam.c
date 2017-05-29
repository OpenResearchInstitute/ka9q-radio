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

void *cam_cleanup(void *arg){
  if(Demod.filter != NULL){
    delete_filter(Demod.filter);
    Demod.response = NULL;
    Demod.filter = NULL;
  }
  return NULL;
}


void demod_cam(void *arg){
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
  gain = 1.0;

  // Set up pre-demodulation filter
  Demod.response = (complex float *)fftwf_alloc_complex(N);
  // posix_memalign((void **)&Demod.response,16,N*sizeof(complex float));
  memset(Demod.response,0,N*sizeof(*Demod.response));
  for(n=low; n <= high; n++)
    Demod.response[(n+N)%N] = gain;
  
  window_filter(Demod.L,Demod.M,Demod.response,Kaiser_beta);
  Demod.decimate = Demod.samprate / Audio.samprate;
  
  Demod.filter = create_filter(Demod.L,Demod.M,Demod.response,Demod.decimate,COMPLEX);
  //  audio_change_parms(Demod.samprate/Demod.decimate,Modes[mode].channels,Demod.L);
  audio_change_parms(Audio.samprate,2,Demod.filter->blocksize_out);  

  pthread_cleanup_push(cam_cleanup,&Demod);

  while(1){
    complex float *buffer;
    read(Demod.data_sock,&buffer,sizeof(buffer));
    
    memcpy(Demod.filter->input,buffer,Demod.filter->blocksize_in*sizeof(*buffer));

    spindown(Demod.filter->input,Demod.filter->blocksize_in); // 2nd LO

    int i;
    i = execute_filter(Demod.filter);
    assert(i == 0);
    // Automatic gain control
    complex float phase;
    int n;
    float audio[Demod.filter->blocksize_out];
    static complex float lastphase;
    double freqerror;
    
    phase = 0;
    for(n=0; n < Demod.filter->blocksize_out; n++)
      phase += Demod.filter->output.c[n];

    phase = conj(phase) / cabs(phase);

    // Rotate signal onto I axis, measure DC (carrier) level
    Demod.amplitude = 0;
    for(n=0; n < Demod.filter->blocksize_out; n++)
      Demod.amplitude += audio[n] = creal(Demod.filter->output.c[n] * phase);
    
    Demod.amplitude /= Demod.filter->blocksize_out;
    float snn = Demod.amplitude / Demod.noise; // (S+N)/N amplitude ratio
    Demod.snr = (snn*snn) -1; // S/N as power ratio


    // Remove carrier DC
    for(n=0; n < Demod.filter->blocksize_out; n++)
      audio[n] -= Demod.amplitude;

    // Frequency error is the phase of this block minus the last, times blocks/sec
    // Phase was already flipped, hence the minus
    // Only move a fraction of the error at one time
    freqerror = -0.01 * carg(phase * conj(lastphase))/(2*M_PI) * Demod.samprate/Demod.filter->blocksize_in;
    lastphase = phase;
    set_second_LO(-freqerror + Demod.second_LO,0);

    // AM AGC is carrier-driven
    Demod.gain = Headroom / Demod.amplitude;
    put_mono_audio(audio,Demod.filter->blocksize_out,Demod.gain); // we do our own
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}
