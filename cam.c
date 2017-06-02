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

void *cam_cleanup(void *arg){
  struct demod *demod = arg;
  delete_filter(demod->filter);
  demod->filter = NULL;
  if(Audio.handle){
    snd_pcm_drop(Audio.handle);
    snd_pcm_close(Audio.handle);
    Audio.handle = NULL;
  }
  return NULL;
}


void *demod_cam(void *arg){
  int n;
  int N;
  const float gain = 1.0;  // unity gain
  int low,high;
  struct demod *demod = arg;

  N = demod->L + demod->M - 1;
  enum mode mode = demod->mode;

  demod->foffset = NAN; // not used
  demod->pdeviation = NAN;

  low = N*Modes[mode].low/demod->samprate;
  high = N*Modes[mode].high/demod->samprate;
  if(low > high){
    int t;
    t = low;
    low = high;
    high = t;
  }

  // Set up pre-demodulation filter
  complex float *response = (complex float *)fftwf_alloc_complex(N);
  // posix_memalign((void **)&response,16,N*sizeof(complex float));
  memset(response,0,N*sizeof(*response));
  for(n=low; n <= high; n++)
    response[(n+N)%N] = gain;
  
  window_filter(demod->L,demod->M,response,Kaiser_beta);
  demod->decimate = demod->samprate / Audio.samprate;
  demod->filter = create_filter(demod->L,demod->M,response,demod->decimate,COMPLEX);
  audio_change_parms(Audio.samprate,2,demod->filter->blocksize_out);  

  pthread_cleanup_push(cam_cleanup,demod);

  complex float lastphase = 0;
  while(1){
    fillbuf(demod->data_sock,(char *)demod->filter->input,
	    demod->filter->blocksize_in*sizeof(complex float));
    spindown(demod,demod->filter->input,demod->filter->blocksize_in); // 2nd LO
    execute_filter(demod->filter);

    // Automatic gain control
    complex float phase;
    int n;
    float audio[demod->filter->blocksize_out];

    double freqerror;
    
    phase = 0;
    for(n=0; n < demod->filter->blocksize_out; n++)
      phase += demod->filter->output.c[n];

    phase = conj(phase) / cabs(phase);

    // Rotate signal onto I axis, measure DC (carrier) level
    demod->amplitude = 0;
    for(n=0; n < demod->filter->blocksize_out; n++)
      demod->amplitude += audio[n] = creal(demod->filter->output.c[n] * phase);
    
    demod->amplitude /= demod->filter->blocksize_out;
    float snn = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio
    demod->snr = (snn*snn) -1; // S/N as power ratio

    // Remove carrier DC
    for(n=0; n < demod->filter->blocksize_out; n++)
      audio[n] -= demod->amplitude;

    // Frequency error is the phase of this block minus the last, times blocks/sec
    // Phase was already flipped, hence the minus
    // Only move a fraction of the error at one time
    freqerror = -0.01 * carg(phase * conj(lastphase))/(2*M_PI) * demod->samprate/demod->filter->blocksize_in;
    lastphase = phase;
    set_second_LO(demod,-freqerror + demod->second_LO,0);

    // AM AGC is carrier-driven
    demod->gain = Headroom / demod->amplitude;
    put_mono_audio(audio,demod->filter->blocksize_out,demod->gain); // we do our own
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}
