#ifndef _DECIMATE_H
#define _DECIMATE_H 1

#define MAX_L 64 // Must be power of 2

struct decimate_state {
  complex float state[MAX_L];
  int ptr;
  float coefficients[MAX_L]; // one-sided array of real coefficients **starting with #1**; #0 is assumed unity
  int ntapso2; // Number of FIR taps divided by two and rounded down
};

#define N 4 // Number of decimation stages; 6 -> 2^6 = 64:1 etc

extern struct decimate_state Filters[N];
int decimate_setup(float beta);
complex float decimate(struct decimate_state *fs,float real, float imag);

#endif
