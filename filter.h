#ifndef _FILTER_H
#define _FILTER_H 1

#include <complex.h>
#include <fftw3.h>
#undef I

extern float Kaiser_beta;

enum filtertype {
  NONE,
  COMPLEX,
  CROSS_CONJ,
  REAL,
};

union rc {
  float *r;
  complex float *c;
};

struct filter {
  enum filtertype in_type;
  enum filtertype out_type;
  int ilen;
  int impulse_length;
  complex float *response;           // Filter response in frequency domain
  complex float *fdomain;            // Signal in frequency domain
  complex float *f_fdomain;          // Filtered signal in frequency domain
  union rc input_buffer;             // Actual time-domain input buffer, length N
  union rc input;                    // Beginning of user input area, length L
  union rc output_buffer;
  union rc output;
  fftwf_plan fwd_plan;
  fftwf_plan rev_plan;
  int decimate;
  int olen;
};
int window_filter(const int L,const int M,complex float *response,const float beta);
int window_rfilter(const int L,const int M,complex float *response,const float beta);

struct filter *create_filter(const int,const int,complex float *,const int,const enum filtertype,const enum filtertype);
int execute_filter(struct filter *);
int execute_filter_nocopy(struct filter *);
int delete_filter(struct filter *);
int make_kaiser(float *window,const int M,const float beta);
#endif
