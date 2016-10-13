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

// FM demodulator
float squelch_fm(complex float *buffer,int L){
  float fm_variance,fm_amplitude;
  float magnitudes[L];
  int n;
  
  // Find average amplitude
  fm_amplitude = 0;
  for(n=0;n<L;n++)
    fm_amplitude += magnitudes[n] = cabsf(buffer[n]);
  fm_amplitude /= L;
  Demod.amplitude = fm_amplitude;
  
  // Find variance
  fm_variance = 0;
  for(n=0;n<L;n++){
    float deviation = magnitudes[n] - fm_amplitude;
    fm_variance += deviation * deviation;
  }
  fm_variance /= (L-1);
  // Find SNR, apply squelch
  return fm_amplitude * fm_amplitude/(2*fm_variance);
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

