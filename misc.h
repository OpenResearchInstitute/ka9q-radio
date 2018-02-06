// $Id: dsp.h,v 1.17 2017/09/28 22:03:47 karn Exp karn $
#ifndef _DSP_H
#define _DSP_H 1

#include <complex.h>
#undef I

#include <math.h> // Get M_PI

#define M_1_2PI (0.5 * M_1_PI) // fraction of a rotation in one radian

complex float const csincosf(const float x);
complex double const csincos(const double x);

float const cpower(const complex float *data, const int len);

// Compute norm = x * conj(x) = Re{x}^2 + Im{x}^2
float const cnrmf(const complex float x);
double const cnrm(const complex double x);

int is_phasor_init(const complex double x);

int pipefill(int,void *,const int);
double const parse_frequency(const char *);
void chomp(char *);


#define max(x,y) ((x) > (y) ? (x) : (y))
#define min(x,y) ((x) < (y) ? (x) : (y))

#define dB2power(x) (powf(10.,(x)/10.))
#define power2dB(x) (10*log10f(x))
#define dB2voltage(x) (powf(10.,(x)/20.))
#define voltage2dB(x) (20*log10f(x))

#define DEGPRA (180./M_PI)
#define RAPDEG (M_PI/180.)

// I *hate* this sort of pointless, stupid, gratuitous incompatibility that
// makes a lot of code impossible to read and debug
// The Linux version of pthread_setname_np takes two args, the OSx version only one
// The GNU malloc_usable_size() does exactly the same thing as the BSD/OSX malloc_size()
// except that the former is defined in <malloc.h>, the latter is in <malloc/malloc.h>
#ifdef __APPLE__
#define pthread_setname(x) pthread_setname_np(x)
#include <malloc/malloc.h>
#define malloc_usable_size(x) malloc_size(x)
#else
#include <malloc.h>
#define pthread_setname(x) pthread_setname_np(pthread_self(),x)
#endif


#endif
