// $Id: misc.c,v 1.20 2017/09/28 22:04:12 karn Exp karn $
// Miscellaneous low-level DSP routines
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1 // Needed to get sincos/sincosf
#endif
#include <complex.h>
#undef I
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#ifndef NULL
#define NULL ((void *)0)
#endif

#include "radio.h"

// return unit magnitude complex number with phase x radians
complex float const csincosf(const float x){
  float s,c;

#if __APPLE__ // No sincos
  s = sinf(x);
  c = cosf(x);
#else
  sincosf(x,&s,&c);
#endif
  return CMPLXF(c,s);
}

// return unit magnitude complex number with phase x radians
complex double const csincos(const double x){
  double s,c;

#if __APPLE__
  s = sin(x);
  c = cos(x);
#else
  sincos(x,&s,&c);
#endif
  return CMPLX(c,s);
}

float const cnrmf(const complex float x){
  return crealf(x)*crealf(x) + cimagf(x) * cimagf(x);
}
double const cnrm(const complex double x){
  return creal(x)*creal(x) + cimag(x) * cimag(x);
}

// Return 1 if phasor appears to be initialized, 0 if not
int is_phasor_init(const complex double x){
  if(isnan(creal(x)) || isnan(cimag(x)) || cnrm(x) < 0.9)
    return 0;
  return 1;
}


// Average power in an array of complex floats
const float cpower(const complex float *data, const int len){
  float amplitude = 0;
  int n;

  if(len <= 0)
    return 0;
  for(n=0; n < len; n++)
    amplitude += cnrmf(data[n]);

  return amplitude/len;
}

// Fill buffer from pipe
// Needed because reads from a pipe can be partial
int pipefill(const int fd,void *buffer,const int cnt){
  int i;
  unsigned char *bp = buffer;
  for(i=0;i<cnt;){
    int n = read(fd,bp+i,cnt-i);
    if(n < 0)
      return n;
    if(n == 0)
      break;
    i += n;
  }
  return i;

}

// Remove return or newline, if any, from end of string
void chomp(char *s){

  if(s == NULL)
    return;
  char *cp;
  if((cp = strchr(s,'\r')) != NULL)
    *cp = '\0';
  if((cp = strchr(s,'\n')) != NULL)
    *cp = '\0';
}


// Parse a frequency entry in the form
// 12345 (12345 Hz)
// 12k345 (12.345 kHz)
// 12m345 (12.345 MHz)
// 12g345 (12.345 GHz)
// If no g/m/k and number is too small, make a heuristic guess
// NB! This assumes radio covers 100 kHz - 2 GHz; should make more general
double const parse_frequency(const char *s){
  char * const ss = alloca(strlen(s));

  int i;
  for(i=0;i<strlen(s);i++)
    ss[i] = tolower(s[i]);

  ss[i] = '\0';
  
  // k, m or g in place of decimal point indicates scaling by 1k, 1M or 1G
  char *sp;
  double mult;
  if((sp = strchr(ss,'g')) != NULL){
    mult = 1e9;
    *sp = '.';
  } else if((sp = strchr(ss,'m')) != NULL){
    mult = 1e6;
    *sp = '.';
  } else if((sp = strchr(ss,'k')) != NULL){
    mult = 1e3;
    *sp = '.';
  } else
    mult = 1;

  char *endptr = NULL;
  double f = strtod(ss,&endptr);
  if(endptr == ss || f == 0)
    return 0; // Empty entry, or nothing decipherable
  
  if(mult != 1 || f >= 1e5) // If multiplier given, or frequency >= 100 kHz (lower limit), return as-is
    return f * mult;
    
  // If frequency would be out of range, guess kHz or MHz
  if(f < 100)
    f *= 1e6;              // 0.1 - 99.999 Only MHz can be valid
  else if(f < 500)         // Could be kHz or MHz, arbitrarily assume MHz
    f *= 1e6;
  else if(f < 2000)        // Could be kHz or MHz, arbitarily assume kHz
    f *= 1e3;
  else if(f < 100000)      // Can only be kHz
    f *= 1e3;

  return f;
}
