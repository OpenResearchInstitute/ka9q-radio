// $Id: fm.c,v 1.21 2017/07/02 12:02:14 karn Exp karn $
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
    float const magsq = cnrmf(buffer[n]); // I^2 + Q^2
    avg_squares += magsq;
    avg += sqrtf(magsq);            // magnitude
  }
  avg_squares /= L; // Average square magnitude
  avg /= L;         // Average magnitude
  demod->amplitude = avg;
  float const fm_variance = avg_squares - avg*avg;
  demod->noise = sqrtf(fm_variance);
  
  // Find SNR
  return avg*avg/(2*fm_variance) - 1;
}


// in-place FM demodulator
// Note: output can range from -PI to +PI; do gain scaling appropriately
int do_fm(float *output,complex float const *buffer,int L,float *state){
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
  float const dsamprate = demod->samprate / demod->decimate; // Decimated (output) sample rate
  float lastaudio = 0;

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

    fillbuf(demod->input,filter->input,filter->ilen*sizeof(complex float));
    spindown(demod,filter->input,filter->ilen); // 2nd LO
    execute_filter(filter);

    // If squelch is closed, just let the output drain
    demod->snr = fm_snr(demod,filter->output.c,filter->olen);
    if(demod->snr > 2){
      float audio[filter->olen];
      int n;
      float avg = 0;
      float ampl = demod->amplitude * demod->amplitude;
      for(n=0; n<filter->olen; n++){
	complex float const prod = filter->output.c[n] * state;
	if(cabsf(prod) > 0.5 * ampl)
	  audio[n] = carg(prod);
	else if(n != 0) // Discard weak sample
	  audio[n] = audio[n-1];
	else
	  audio[n] = lastaudio;
	state = conjf(filter->output.c[n]);
	avg += audio[n];
      }
      lastaudio = audio[n-1];
      avg /= filter->olen;

      // Find peak deviation, scale for output
      float pdev = 0;
      for(n=0;n<filter->olen;n++){
	if(fabsf(audio[n] - avg) > pdev)
	  pdev = fabsf(audio[n] - avg);
	audio[n] *= demod->gain;
      }
      demod->foffset = (avg / (2*M_PI)) * (demod->samprate / demod->decimate);
      demod->pdeviation = (pdev / (2*M_PI)) * (demod->samprate / demod->decimate);
      
      send_mono_audio(audio,n);
    }
  }
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}

