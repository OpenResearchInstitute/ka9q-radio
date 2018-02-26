#define _GNU_SOURCE 1
#include <stdio.h>
#include <stdlib.h>
#include <complex.h>
#include <math.h>

int popcount(unsigned int n){
  int c = 0;

  while(n != 0){
    c += n & 1;
    n >>= 1;
  }
  return c;
}

int parity(unsigned int n){
  return popcount(n) & 1;
}



void putsample(double s){
  double const Scale = 32767;

  signed short sym = Scale * s;
  putchar(sym & 0xff);
  putchar((sym >> 8) & 0xff);
}


int main(){
  unsigned int cstate = 0;

  int const poly1 = 0171;
  int const poly2 = 0133; // This one is inverted
  complex double cphase = 1;
  //  complex double cstep = cexp(2*M_PI*I*2400./48000.);
  complex double cstep = 1;

  for(int n=0; n<1000000; n++){
    int bit;
    int sym1,sym2;


    if((n % 1200) < 100){
      sym1 = sym2 = 0;
    } else if((n % 1200) < 200){
      sym1 = 0; sym2 = 1;
    } else {
      bit = random() & 1;
      cstate |= bit << 6;
      sym1 = parity(cstate & poly1);
      sym2 = !parity(cstate & poly2);
      cstate = cstate >> 1;
    }
    for(int k=0; k < 20; k++){
      putsample(sym1 ? cphase : -cphase);
      cphase *= cstep;
    }
    for(int k=0; k < 20; k++){
      putsample(sym2 ? cphase : -cphase);
      cphase *= cstep;
    }
  }
  exit(0);
}
