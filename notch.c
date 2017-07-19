#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include "dsp.h"


struct notchfilter *notch_create(double f,float tc){
  struct notchfilter *nf = calloc(1,sizeof(struct notchfilter));
  nf->osc_phase = 1;
  nf->osc_step = csincos(2*M_PI*f);
  nf->dcstate = 0;
  nf->tc = tc;
  return nf;
}

void notch_delete(struct notchfilter *nf){
  free(nf);
}



complex float notch(struct notchfilter *nf,complex float s){
  s *= conj(nf->osc_phase);    // Spin down to DC
  s -= nf->dcstate;            // Remove DC
  nf->dcstate += nf->tc * s;   // Update smoothed estimate
  s *= nf->osc_phase;          // Spin back up
  nf->osc_phase *= nf->osc_step;
  return s;
}
