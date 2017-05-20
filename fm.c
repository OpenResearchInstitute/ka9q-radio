// $Id: fm.c,v 1.2 2016/10/13 23:29:54 karn Exp karn $
// FM demodulation and squelch
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#undef I

#include "dsp.h"
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

