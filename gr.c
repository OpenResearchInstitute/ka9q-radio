// $Id: gr.c,v 1.2 2016/10/13 23:34:07 karn Exp karn $
// Mirics default gain reduction tables
// Mirics SDR API specification, section 5
#include <assert.h>
#include "dsp.h"
int mirics_gain(double f,int gr,int *bb, int *lna,int *mix){
  *bb = *lna = *mix = 0;
  if(gr < 0)
    gr = 0;

  if(f < 60e6){
    if(gr >= 35)
      *lna = 24;
  } else if(f < 120e6){
    // Same as < 60 MHz, why is this separate?
    if(gr >= 35)
      *lna = 24;
  } else if(f < 250e6){
    if(gr >= 29)
      *lna = 24;
  } else if(f < 420e6){
    if(gr >= 35)
      *lna = 24;
  } else if(f < 1e9){
    if(gr >= 12)
      *lna = 7;
  } else { // 1-2 GHz
    if(gr >= 10)
      *lna = 7;
  }
  // If the IF can't handle the rest, turn off the mixer too
  gr -= *lna;
  if(gr > 59)
    *mix = 19;
  gr -= *mix;
  *bb = min(gr,59);
  return 0;
}
