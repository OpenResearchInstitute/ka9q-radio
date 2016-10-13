#include <stdio.h>
#include <stdlib.h>
#include "sdr.h"

int front_end_init(int samprate,int L){
  return 0;
}

double set_first_LO(double f){
  return 0;
}
double get_first_LO(void) {
  return 0;
}
int get_nominal_samprate(void) {
  //  return 6144000;
    return 1536000;
}
double get_exact_samprate(void) {
    return 1536000;
    //return 6144000;
}
double set_sdr_cal(double x,int relative) {
  return 0;
}
float get_adc(complex float *buffer,int L) {
  int i;
  short samps[2*L];
  float energy;


  if(feof(stdin))
    exit(0);
  fread(samps,sizeof(short),2*L,stdin);
  energy = 0;
  for(i=0;i<L;i++){
    buffer[i] = CMPLXF(samps[2*i],samps[2*i+1]);
    energy += crealf(buffer[i])*crealf(buffer[i]);
    energy += cimagf(buffer[i])*cimagf(buffer[i]);    
  }
  return energy/L;
}
void closedown(int a) {
}

