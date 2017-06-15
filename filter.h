#ifndef _FILTER_H
#define _FILTER_H 1

#include <complex.h>
#include <fftw3.h>
#undef I

extern double Kaiser_beta;

enum filtertype {
  NONE,
  COMPLEX,
  CROSS_CONJ,
  REAL,
};

struct filter {
  enum filtertype type;
  int blocksize_in;
  int impulse_length;
  complex float *response;           // Filter response in frequency domain
  complex float *fdomain;            // Signal in frequency domain
  complex float *input_buffer;       // Actual time-domain input buffer, length N
  complex float *input;              // Beginning of user input area, length L
  union {
    float *r;
    complex float *c;
    void *v;
  } output_buffer;
  union {
    float *r;
    complex float *c;
    void *v;
  } output;
  fftwf_plan fwd_plan;
  fftwf_plan rev_plan;
  int decimate;
  int blocksize_out;
};
int window_filter(const int L,const int M,complex float *response,const double beta);
int window_rfilter(const int L,const int M,complex float *response,const double beta);

struct filter *create_filter(const int,const int,complex float *,const int,const enum filtertype);
int execute_filter(struct filter *);
int delete_filter(struct filter *);
#endif
