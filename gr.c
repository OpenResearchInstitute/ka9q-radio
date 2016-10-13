#include <assert.h>
#include "dsp.h"
// Mirics default gain reduction tables
// Mirics SDR API specification, section 5
int mirics_gain(double f,int gr,int *bb, int *lna,int *mix){
  *bb = *lna = *mix = 0;
  if(gr < 0)
    gr = 0;

  if(f < 60e6){
    if(gr >= 35)
      *lna = 24;
    if(gr >= 83)
      *mix = 19;
  } else if(f < 120e6){
    // Same as < 60 MHz, why is this separate?
    if(gr >= 35)
      *lna = 24;
    if(gr >= 83)
      *mix = 19;
  } else if(f < 250e6){
    if(gr >= 29)
      *lna = 24;
    if(gr >= 83)
      *mix = 19;
  } else if(f < 420e6){
    if(gr >= 35)
      *lna = 24;
    if(gr >= 83)
      *mix = 19;
  } else if(f < 1e9){
    if(gr >= 12)
      *lna = 7;
    if(gr >= 66)
      *mix = 19;
  } else { // 1-2 GHz
    if(gr >= 10)
      *lna = 7;
    if(gr >= 66)
      *mix = 19;
  }
  // Whatever the LNA and mixer don't do, the IF does
  *bb = min(gr - *lna - *mix,59);
  return 0;
}
