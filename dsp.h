// $Id: dsp.h,v 1.20 2018/12/11 11:42:11 karn Exp karn $
// Low-level, ostly math functions useful for digital signal processing
#ifndef _DSP_H
#define _DSP_H 1

#include <complex.h>
#undef I

#include <math.h> // Get M_PI

#define M_1_2PI (0.5 * M_1_PI) // fraction of a rotation in one radian

#if __APPLE__
#define sincos(x,s,c) __sincos(x,s,c)
#define sincosf(x,s,c) __sincosf(x,s,c)
#define sincospi(x,s,c) __sincospi(x,s,c)
#define sincospif(x,s,c) __sincospif(x,s,c)
#else
#define sincospi(x,s,c) sincos(x*M_PI,s,c)
#define sincospif(x,s,c) sincosf(x*M_PI,s,c)
#endif


// Cos(x) + j*sin(x)
#define cisf(x) csincosf(x)
#define cispif(x) csincospif(x)
#define cis(x) csincos(x)
#define cispi(x) csincospi(x)

inline complex float const csincosf(const float x){
  float s,c;

  sincosf(x,&s,&c);
  return CMPLXF(c,s);
}
inline complex float const csincospif(const float x){
  float s,c;
  sincospif(x,&s,&c);
  return CMPLXF(c,s);

}

// return unit magnitude complex number with given phase x
inline complex double const csincos(const double x){
  double s,c;

  sincos(x,&s,&c);
  return CMPLX(c,s);
}
inline complex double const csincospi(const double x){
  double s,c;
  sincospi(x,&s,&c);
  return CMPLX(c,s);
}


// Complex norm (sum of squares of real and imaginary parts)
inline float const cnrmf(const complex float x){
  return crealf(x)*crealf(x) + cimagf(x) * cimagf(x);
}
inline double const cnrm(const complex double x){
  return creal(x)*creal(x) + cimag(x) * cimag(x);
}


#define dB2power(x) (powf(10.,(x)/10.))
#define power2dB(x) (10*log10f(x))
#define dB2voltage(x) (powf(10.,(x)/20.))
#define voltage2dB(x) (20*log10f(x))

#define DEGPRA (180./M_PI)
#define RAPDEG (M_PI/180.)

#endif

