// $Id: osc.c,v 1.5 2019/01/14 12:41:57 karn Exp karn $
// Complex oscillator object routines

#define _GNU_SOURCE 1
#include <assert.h>
#include <math.h>
#include <complex.h>
#include "osc.h"
#include "dsp.h"

const int Renorm_rate = 16384; // Renormalize oscillators this often

// Return 1 if complex phasor appears to be initialized, 0 if not
int is_phasor_init(const complex double x){
  if(isnan(creal(x)) || isnan(cimag(x)) || cnrm(x) < 0.9)
    return 0;
  return 1;
}

// Set oscillator frequency and sweep rate
// Units are cycles/sample and cycles/sample^2
void set_osc(struct osc *osc,double f,double r){
  pthread_mutex_lock(&osc->mutex);
  if(!is_phasor_init(osc->phasor)){
    osc->phasor = 1; // Don't jump phase if already initialized
    osc->steps = 0;
  }
  osc->freq = f;
  osc->rate = r;
  osc->phasor_step = cispi(2 * osc->freq);
  if(osc->rate != 0)
    osc->phasor_step_step = cispi(2 * osc->rate);
  else
    osc->phasor_step_step = 1;
  pthread_mutex_unlock(&osc->mutex);
}

// Step oscillator through one sample, return complex phase
complex double step_osc(struct osc *osc){
  complex double r;

  r = osc->phasor;
  if(osc->rate != 0)
    osc->phasor_step *= osc->phasor_step_step;

  osc->phasor *= osc->phasor_step;
  if(++osc->steps == Renorm_rate)
    renorm_osc(osc);
  return r;
}

void renorm_osc(struct osc *osc){
  osc->steps = 0;
  osc->phasor /= cabs(osc->phasor);

  if(osc->rate != 0)
    osc->phasor_step /= cabs(osc->phasor_step);
}
#include <stdio.h>
#include <stdlib.h>


// Initialize digital phase lock loop with bandwidth, damping factor, initial VCO frequency and sample rate
void init_pll(struct pll *pll,float nf,float damping,double freq,float samprate){

  pll->samptime = 1./samprate;
  freq *= pll->samptime; // initial VCO frequency in cycles/sample
  nf *= pll->samptime;  // natural frequency in cycles/sample

  // Second-order PLL loop filter (see Gardner)
  float const vcogain = 2*M_PI;            // 2 pi radians/sample per "volt"
  float const pdgain = 1;                  // phase detector gain "volts" per radian (unity from atan2)
  float const natfreq = nf * 2*M_PI;       // loop natural frequency in rad/sample
  float const tau1 = vcogain * pdgain / (natfreq * natfreq);
  float const tau2 = 2 * damping / natfreq;

  pll->prop_gain = tau2 / tau1;
  pll->integrator_gain = 1 / tau1;
  pll->integrator = freq * pll->samptime / pll->integrator_gain; // To give specified frequency
#if 0
  fprintf(stderr,"init_pll(%p,%f,%f,%f,%f)\n",pll,nf,damping,freq,samprate);
  fprintf(stderr,"natfreq %lg tau1 %lg tau2 %lg propgain %lg intgain %lg\n",
	  natfreq,tau1,tau2,pll->prop_gain,pll->integrator_gain);
#endif
}


// Step the PLL through one sample, return VCO control voltage
// Initial implementation, will probably be slow because of the sincos() for every sample
// Return PLL freq in cycles/sample
float run_pll(struct pll *pll,float phase){

  float feedback = pll->integrator_gain * pll->integrator + pll->prop_gain * phase;
  pll->integrator += phase;
  
  feedback = feedback > 0.49 ? 0.49 : feedback < -0.49 ? -0.49 : feedback;
  set_osc(&pll->vco,feedback,0); // This may be a CPU problem on every sample
  step_osc(&pll->vco);
#if 0
  if((random() & 0xffff) == 0){
    fprintf(stderr,"phase %f integrator %g feedback %g pll_freq %g\n",
	    phase,pll->integrator,feedback,feedback/pll->samptime);
  }
#endif
   
  return feedback;
}

