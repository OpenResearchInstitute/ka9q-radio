// $Id: fm.c,v 1.20 2017/07/02 04:29:51 karn Exp karn $
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

// Estimate FM SNR
float fm_snr(struct demod *demod,complex float *buffer,int L){
  // Find average magnitude and magnitude^2
  // Approximate because magnitude has a chi-squared distribution with 2 degrees of freedom
  float avg_squares = 0;
  float avg = 0;
  int n;
  for(n=0;n<L;n++){
    const float magsq = cnrmf(buffer[n]); // I^2 + Q^2
    avg_squares += magsq;
    avg += sqrtf(magsq);            // magnitude
  }
  avg_squares /= L; // Average square magnitude
  avg /= L;         // Average magnitude
  demod->amplitude = avg;
  const float fm_variance = avg_squares - avg*avg;
  
  // Find SNR
  return avg*avg/(2*fm_variance) - 1;
}


// in-place FM demodulator
// Note: output can range from -PI to +PI; do gain scaling appropriately
int do_fm(float *output,const complex float *buffer,int L,complex float *state){
  int n;

  output[0] = cargf(buffer[0] * conjf(*state));
  for(n=1; n<L; n++)
    output[n] = cargf(buffer[n] * conj(buffer[n-1]));

  *state = buffer[L-1];
  return 0;
}



void *demod_fm(void *arg){
  assert(arg != NULL);
  struct demod * const demod = arg;  
  complex float state = 0;
  pthread_setname_np(pthread_self(),"fm");
  const float dsamprate = demod->samprate / demod->decimate; // Decimated (output) sample rate

  demod->pdeviation = 0;
  demod->foffset = 0;

  // Create filter, leaving response momentarily empty
  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX);
  demod->filter = filter;
  set_filter(demod,demod->low,demod->high); // Set response

  while(!demod->terminate){
    // Constant gain used by FM only; automatically adjusted by AGC in linear modes
    // We do this in the loop because BW can change
    demod->gain = (Headroom *  M_1_PI * dsamprate) / fabsf(demod->low - demod->high);

    fillbuf(demod->input,(char *)filter->input,
	    filter->blocksize_in*sizeof(complex float));
    spindown(demod,filter->input,filter->blocksize_in); // 2nd LO
    execute_filter(filter);

    // If squelch is closed, just let the output drain
    demod->snr = fm_snr(demod,filter->output.c,filter->blocksize_out);
    if(demod->snr > 2){
      float audio[filter->blocksize_out];
      do_fm(audio,filter->output.c,filter->blocksize_out,&state);

      float pdev = 0;
      float avg = 0;

      int n;
      for(n=0;n<filter->blocksize_out;n++)
	avg += audio[n];

      avg /= filter->blocksize_out;

      for(n=0;n<filter->blocksize_out;n++){
	if(fabsf(audio[n] - avg) > pdev)
	  pdev = fabsf(audio[n] - avg);
      }
      demod->foffset = (avg / (2*M_PI)) * (demod->samprate / demod->decimate);
      demod->pdeviation = (pdev / (2*M_PI)) * (demod->samprate / demod->decimate);
      
      // Scale and send to audio thread
      for(n=0;n<filter->blocksize_out;n++)
	audio[n] *= demod->gain;
      
      send_mono_audio(audio,n);
    }
  }
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}

