// $Id: dsp.h,v 1.3 2016/10/14 04:34:21 karn Exp karn $
#ifndef _DSP_H
#define _DSP_H 1

#include <complex.h>
#undef I

const complex float csincosf(float x);
const complex double csincos(double x);

float amplitude(const float *data,int len);
float camplitude(const complex float *data, int len);

// Compute norm = x * conj(x) = Re{x}^2 + Im{x}^2
const float cnrmf(complex float x);
const double cnrm(complex double x);

void verify(complex float *,int);

int fillbuf(const int,char *,int);

#define CLIP(x) ((x) > SHRT_MAX ? SHRT_MAX : (x) < SHRT_MIN ? SHRT_MIN : (x))
#define max(x,y) ((x) > (y) ? (x) : (y))
#define min(x,y) ((x) < (y) ? (x) : (y))

#define dB2power(x) (powf(10.,(x)/10.))
#define power2dB(x) (10*log10f(x))
#define dB2voltage(x) (powf(10.,(x)/20.))
#define voltage2dB(x) (20*log10f(x))

#define DEGPRA (180./M_PI)
#define RAPDEG (M_PI/180.)

#endif
