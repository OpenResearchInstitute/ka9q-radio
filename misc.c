// $Id: misc.c,v 1.6 2017/06/02 12:06:07 karn Exp karn $
// Miscellaneous low-level DSP routines
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1 // Needed to get sincos/sincosf
#endif
#include <complex.h>
#undef I
#include <math.h>
#include <assert.h>
#include <unistd.h>

#ifndef NULL
#define NULL ((void *)0)
#endif

// return unit magnitude complex number with phase x radians
const complex float csincosf(const float x){
  float s,c;

  sincosf(x,&s,&c);
  return CMPLXF(c,s);
}

// return unit magnitude complex number with phase x radians
const complex double csincos(const double x){
  double s,c;

  sincos(x,&s,&c);
  return CMPLX(c,s);
}

const float cnrmf(const complex float x){
  return crealf(x)*crealf(x) + cimagf(x) * cimagf(x);
}
const double cnrm(const complex double x){
  return creal(x)*creal(x) + cimag(x) * cimag(x);
}

// Root-mean-square of an array of floats
const float amplitude(const float *data,const int len){
  float sum = 0;  
  int n;
  
  if(len <= 0)
    return 0;
  for(n=0; n < len; n++)
    sum += data[n] * data[n];
  return sqrtf(sum/len);
}

// Root-mean-square of the magnitudes of an array of complex floats
const float camplitude(const complex float *data, const int len){
  float amplitude = 0;
  int n;

  if(len <= 0)
    return 0;
  for(n=0; n < len; n++)
    amplitude += cnrmf(data[n]);

  return sqrtf(amplitude/len);
}

int fillbuf(const int fd,char *buffer,const int cnt){
  int i;
  for(i=0;i<cnt;){
    int n;
    
    n = read(fd,&buffer[i],cnt-i);
    if(n < 0)
      return n;
    i += n;
  }
  return cnt;

}

