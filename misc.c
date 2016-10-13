// $Id$
// Miscellaneous low-level DSP routines
#define _GNU_SOURCE 1
#include <complex.h>
#include <math.h>
#include <assert.h>

#ifndef NULL
#define NULL ((void *)0)
#endif

// return unit magnitude complex number with phase x radians
const complex float csincosf(float x){
  float s,c;

  sincosf(x,&s,&c);
  return CMPLXF(c,s);
}

// return unit magnitude complex number with phase x radians
const complex double csincos(double x){
  double s,c;

  sincos(x,&s,&c);
  return CMPLX(c,s);
}

const float cnrmf(complex float x){
  return crealf(x)*crealf(x) + cimagf(x) * cimagf(x);
}
const double cnrm(complex double x){
  return creal(x)*creal(x) + cimag(x) * cimag(x);
}

float amplitude(const float *data,int len){
  float sum = 0;  
  int n;
  
  if(len <= 0)
    return 0;
  for(n=0; n < len; n++)
    sum += data[n] * data[n];
  return sqrtf(sum/len);
}
float camplitude(const complex float *data, int len){
  float amplitude = 0;
  int n;

  if(len <= 0)
    return 0;
  for(n=0; n < len; n++)
    amplitude += cnrmf(data[n]);

  return sqrtf(amplitude/len);
}



