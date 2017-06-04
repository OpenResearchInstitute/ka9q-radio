// $Id: dsp.h,v 1.4 2017/06/02 12:05:45 karn Exp karn $
#ifndef _DSP_H
#define _DSP_H 1

#include <complex.h>
#undef I

const complex float csincosf(const float x);
const complex double csincos(const double x);

const float amplitude(const float *data,const int len);
const float camplitude(const complex float *data, const int len);

// Compute norm = x * conj(x) = Re{x}^2 + Im{x}^2
const float cnrmf(const complex float x);
const double cnrm(const complex double x);

int fillbuf(const int,char *,const int);

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
