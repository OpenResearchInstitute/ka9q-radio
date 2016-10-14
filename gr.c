// $Id: gr.c,v 1.3 2016/10/14 01:17:27 karn Exp karn $
// Mirics default gain reduction tables
// Mirics SDR API specification, section 5
int mirics_gain(double f,int gr,int *bb, int *lna,int *mix){
  if(gr < 0)
    gr = 0;

  *bb = *lna = *mix = 0;
  if(f < 120e6){ // Also for < 60 MHz in original reference
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
  if(gr > 59){
    *mix = 19;
    gr -= 19;
  }
  *bb = gr > 59 ? 59 : gr;
  return *bb + *mix + *lna;
}
