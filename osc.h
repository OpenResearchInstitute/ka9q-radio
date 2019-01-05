#ifndef _OSC_H
#define _OSC_H 1

#define _GNU_SOURCE 1
#include <pthread.h>
#include <math.h>
#include <complex.h>

struct osc {
  double freq;
  double rate;
  complex double phasor;
  complex double phasor_step;
  complex double phasor_step_step;
  pthread_mutex_t mutex;
  int steps; // Steps since last normalize
};

struct pll {
  float samptime;
  struct osc vco;
  float integrator_gain;
  float prop_gain;
  float integrator;
};





// Osc functions
void set_osc(struct osc *osc,double f,double r);
complex double step_osc(struct osc *osc);
void renorm_osc(struct osc *osc);
int is_phasor_init(const complex double x);

// PLL functions
void init_pll(struct pll *pll,float bw,float damping,double freq,float samprate);
float run_pll(struct pll *pll,complex float sample);


#endif

