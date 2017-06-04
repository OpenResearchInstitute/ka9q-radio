// $Id: fm.c,v 1.10 2017/06/02 12:06:06 karn Exp karn $
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
#include "audio.h"

// Estimate FM SNR
float fm_snr(struct demod *demod,complex float *buffer,int L){
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
  demod->amplitude = avg;
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


void *demod_fm(void *arg){
  int n;
  int N;
  const float gain = 1.0;  // unity gain
  complex float state = 0;
  int low,high;
  struct demod *demod = arg;
  int devhold = 0;

  enum mode mode = demod->mode;
  N = demod->L + demod->M - 1;

  demod->pdeviation = 0;
  demod->foffset = 0;

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
  // Constant gain used by FM only; automatically adjusted by AGC in linear modes
  demod->gain = (Headroom * N / M_PI) / (demod->decimate * abs(low - high));
  audio_change_parms(Audio.samprate,2,demod->filter->blocksize_out);  
  demod->foffset = 0;

  pthread_cleanup_push(fm_cleanup,demod);

  while(1){
    fillbuf(demod->input,(char *)demod->filter->input,
	    demod->filter->blocksize_in*sizeof(complex float));
    spindown(demod,demod->filter->input,demod->filter->blocksize_in); // 2nd LO
    execute_filter(demod->filter);

    // If squelch is closed, just let the output drain
    demod->snr = fm_snr(demod,demod->filter->output.c,demod->filter->blocksize_out);
    if(demod->snr > 2){
      float audio[demod->filter->blocksize_out];
      do_fm(audio,demod->filter->output.c,demod->filter->blocksize_out,&state);
      for(n=0;n<demod->filter->blocksize_out;n++){
	demod->foffset += 0.00005 * (audio[n] - demod->foffset);
	if(devhold == 0 || (fabs(audio[n] - demod->foffset)) > demod->pdeviation){
	  demod->pdeviation = fabs(audio[n] - demod->foffset);
	  devhold = 1 * demod->samprate/demod->decimate;
	} else {
	  devhold--;
	}
      }
      // Scale, convert to stereo and send to audio thread
      complex float buffer[demod->filter->blocksize_out];
      for(n=0;n<demod->filter->blocksize_out;n++){
	buffer[n] = demod->gain * CMPLXF(audio[n],audio[n]);
      }
      write(demod->output,buffer,sizeof(buffer));
    }
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}

