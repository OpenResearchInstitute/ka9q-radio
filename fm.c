// $Id: fm.c,v 1.19 2017/06/20 03:01:01 karn Exp karn $
// FM demodulation and squelch
#define _GNU_SOURCE 1
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

const float squelchtail = 0; // Seconds to hold squelch open after LOS
//const float squelchtail = 0.0625; // Seconds to hold squelch open after LOS

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

void fm_cleanup(void *arg){
  struct demod *demod = arg;
  delete_filter(demod->filter);
  demod->filter = NULL;
}


void *demod_fm(void *arg){
  complex float state = 0;
  struct demod *demod = arg;

  pthread_setname_np(pthread_self(),"fm");
  const float dsamprate = demod->samprate / demod->decimate; // Decimated (output) sample rate

  demod->pdeviation = 0;
  demod->foffset = 0;

  // Create filter, leaving response momentarily empty
  demod->filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX);
  set_filter(demod,demod->low,demod->high); // Set response

  pthread_cleanup_push(fm_cleanup,demod);

  while(1){
    // Constant gain used by FM only; automatically adjusted by AGC in linear modes
    // We do this in the loop because BW can change
    demod->gain = (Headroom *  M_1_PI * dsamprate) / fabsf(demod->low - demod->high);

    fillbuf(demod->input,(char *)demod->filter->input,
	    demod->filter->blocksize_in*sizeof(complex float));
    spindown(demod,demod->filter->input,demod->filter->blocksize_in); // 2nd LO
    execute_filter(demod->filter);

    // If squelch is closed, just let the output drain
    demod->snr = fm_snr(demod,demod->filter->output.c,demod->filter->blocksize_out);
    if(demod->snr > 2){
      float audio[demod->filter->blocksize_out];
      do_fm(audio,demod->filter->output.c,demod->filter->blocksize_out,&state);

      float pdev = 0;
      float avg = 0;

      int n;
      for(n=0;n<demod->filter->blocksize_out;n++)
	avg += audio[n];
      
      avg /= demod->filter->blocksize_out;
      
      for(n=0;n<demod->filter->blocksize_out;n++){
	if(fabsf(audio[n] - avg) > pdev)
	  pdev = fabsf(audio[n] - avg);
      }
      demod->foffset = (avg / (2*M_PI)) * (demod->samprate / demod->decimate);
      demod->pdeviation = (pdev / (2*M_PI)) * (demod->samprate / demod->decimate);
      
      // Scale and send to audio thread
      for(n=0;n<demod->filter->blocksize_out;n++)
	audio[n] *= demod->gain;
      
      send_mono_audio(audio,n);
    }
  }
  pthread_cleanup_pop(1);
  pthread_exit(NULL);
}

