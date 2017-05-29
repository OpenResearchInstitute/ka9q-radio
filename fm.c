// $Id: fm.c,v 1.3 2017/05/20 01:14:08 karn Exp karn $
// FM demodulation and squelch
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#undef I

#include "dsp.h"
#include "filter.h"
#include "radio.h"
#include "fm.h"
#include "audio.h"

// Estimate FM SNR
float fm_snr(complex float *buffer,int L){
  float fm_variance;
  int n;
  
  // Find average magnitude and magnitude^2
  // Approximate because magnitude has a chi-squared distribution with 2 degrees of freedom
  float avg_squares = 0;
  float avg = 0;
  for(n=0;n<L;n++){
    float magsq = cnrmf(buffer[n]); // I^2 + Q^2
    avg_squares += magsq;
    avg += sqrtf(magsq);            // magnitude
  }
  avg_squares /= L; // Average square magnitude
  avg /= L;         // Average magnitude
  Demod.amplitude = avg;
  fm_variance = avg_squares - avg*avg;
  
  // Find SNR
  return avg*avg/(2*fm_variance) - 1;
}


// in-place FM demodulator
// Note: output can range from -PI to +PI; do gain scaling appropriately
int do_fm(float *output,complex float *buffer,int L,complex float *state){
  int n;

  output[0] = cargf(buffer[0] * conjf(*state));
  for(n=1; n<L; n++)
    output[n] = cargf(buffer[n] * conj(buffer[n-1]));

  *state = buffer[L-1];
  return 0;
}

void *fm_cleanup(void *arg){
  if(Demod.filter != NULL){
    delete_filter(Demod.filter);
    Demod.response = NULL;
    Demod.filter = NULL;
  }
  return NULL;
}


void demod_fm(void *arg){
  int n;
  int N;
  float gain = 1.0;  // unity gain
  complex float state = 0;
  int low,high;

  enum mode mode = Demod.mode;
  N = Demod.L + Demod.M - 1;

  low = N*Modes[mode].low/Demod.samprate;
  high = N*Modes[mode].high/Demod.samprate;
  if(low > high){
    int t;
    t = low;
    low = high;
    high = t;
  }
  


  // Set up pre-demodulation filter
  Demod.response = (complex float *)fftwf_alloc_complex(N);
  // posix_memalign((void **)&Demod.response,16,N*sizeof(complex float));
  memset(Demod.response,0,N*sizeof(*Demod.response));
  for(n=low; n <= high; n++)
    Demod.response[(n+N)%N] = gain;
  
  window_filter(Demod.L,Demod.M,Demod.response,Kaiser_beta);
  Demod.decimate = Demod.samprate / Audio.samprate;
  
  Demod.filter = create_filter(Demod.L,Demod.M,Demod.response,Demod.decimate,COMPLEX);
  // Constant gain used by FM only; automatically adjusted by AGC in linear modes
  Demod.gain = (Headroom * N / M_PI) / (Demod.decimate * abs(low - high));
  //  audio_change_parms(Demod.samprate/Demod.decimate,Modes[mode].channels,Demod.L);
  audio_change_parms(Audio.samprate,2,Demod.filter->blocksize_out);  

  pthread_cleanup_push(fm_cleanup,&Demod);

  while(1){
    complex float *buffer;
    read(Demod.data_sock,&buffer,sizeof(buffer));
    
    memcpy(Demod.filter->input,buffer,Demod.filter->blocksize_in*sizeof(*buffer));

    spindown(Demod.filter->input,Demod.filter->blocksize_in); // 2nd LO

    int i;
    i = execute_filter(Demod.filter);
    assert(i == 0);


    Demod.snr = fm_snr(Demod.filter->output.c,Demod.filter->blocksize_out);
    
    // If squelch is closed, just let the output drain
    if(Demod.snr > 2){
      float audio[Demod.filter->blocksize_out];
      do_fm(audio,Demod.filter->output.c,Demod.filter->blocksize_out,&state);
      put_mono_audio(audio,Demod.filter->blocksize_out,Demod.gain);
    }
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}

