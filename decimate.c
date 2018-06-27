// Cascaded half-band filters for sample rate decimation by powers of 2
#include <complex.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include "window.h"
#include "decimate.h"


struct decimate_state Filters[N];


static float sincf(float x){
  if(x == 0)
    return 1;
  return sinf(x) / x;
}


int decimate_setup(float beta){
  memset(Filters,0,sizeof(Filters));

  int ntaps = 4+1;  // Make this depend on n
  for(int n=0; n < N; n++){
    int ntapso2 = ntaps/2; // Will floor down if ntaps is odd, as it should be
  
    float kaiser_coeffs[ntaps];
    // Make full symmetric kaiser window with peak in center
    make_kaiser(kaiser_coeffs,ntaps,beta);

    Filters[n].ntapso2 = ntapso2;
    for(int i=1; i<ntapso2; i++){
      float c = sincf(i*M_PI/2);
      
      // Need to get rid of even coefficients here, they're all zeros
      // Coefficient #1 goes into coefficients[0]
      Filters[n].coefficients[i-1] = c * kaiser_coeffs[i+ntapso2]; // Skip the central coefficient
    }
    ntaps = 2*(ntaps-1) + 1;
  }
  return 0;
}

complex float decimate(struct decimate_state * restrict fs,float real,float imag){
  
  fs = Filters;

  complex float s = CMPLXF(real,imag);

  // Run through series of concatenated half-band filters
  for(int n=0; n<N; n++){
    int ptr = (fs->ptr + 1) & (MAX_L-1);
    fs->ptr = ptr;
    fs->state[(ptr + fs->ntapso2 - 1) & (MAX_L-1)] = s;
    if((ptr & 1) != 0) // Each stage decimates by 2
      return CMPLXF(NAN,NAN); // Not yet ready with sample from output of decimator
    // actually execute and recursively invoke on every other sample to decimate by 2:1
    s = fs->state[ptr]; // 0 (center peak) coefficient is always 1
    // Only do the odd coefficients; even coefficients (except #0) are all zero in a halfband filter
    // Change this so coefficients[] only stores the odd non-zero coefficients
    for(int i = 1; i < fs->ntapso2; i += 2){
      // Impulse response is symmetric, avoid extra multiply
      s += (fs->state[(ptr + i) & (MAX_L-1)]
	    + fs->state[(ptr - i) & (MAX_L-1)]) * fs->coefficients[i-1]; // ensure arg to modulo (%) is positive
    }
    fs++;
  }
  // If we make it entirely through the loop, we've processed 2^N samples; return the new decimator output sample
  return s;
}

#if 0
#include <stdlib.h>
#include <stdio.h>

int main(){
  
  decimate_setup(3.0);

  for(int i=0;;i++){
    complex float s = sinf(i * 2 * M_PI/90);
    
    s = halfband(Filters,s);
    if(!isnan(crealf(s)) && !isnan(cimagf(s))){
      printf("%d %f %f\n",i,crealf(s),cimagf(s));
    }
  }
  exit(0);
}
#endif
