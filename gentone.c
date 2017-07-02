#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <limits.h>
#include "dsp.h"

#define BLOCKSIZE 32768

const int Samprate = 192000;

int main(int argc,char *argv[]){
  int c;

  double frequency;
  double amplitude;
  double sweep;

  amplitude = -20;
  frequency = 48000;
  sweep = 0;

  while((c = getopt(argc,argv,"f:a:s:")) != EOF){
    switch(c){
    case 'f':
      frequency = strtod(optarg,NULL);
      break;
    case 'a':
      amplitude = strtod(optarg,NULL);
      break;
    case 's':
      sweep = strtod(optarg,NULL); // sweep rate, Hz/sec
      break;
    }
  }
  frequency *= 2*M_PI/Samprate;       // radians/sample
  amplitude = pow(10.,amplitude/20.); // Convert to amplitude ratio
  sweep *= 2*M_PI / ((double)Samprate*Samprate);  // radians/sample

  complex double phase_accel = csincos(sweep);
  complex double phase_step = csincos(frequency);
  complex double phase = 1;  
  while(1){
    int i;
    complex float buffer[BLOCKSIZE];
    for(i=0;i<BLOCKSIZE;i++){
      buffer[i] = phase * amplitude;
      phase *= phase_step;
      phase_step *= phase_accel;
    }
    phase /= cabs(phase);
    phase_step /= cabs(phase_step);
    int16_t output[2*BLOCKSIZE];
    for(i=0;i<BLOCKSIZE;i++){
      output[2*i] = crealf(buffer[i]) * SHRT_MAX;
      output[2*i+1] = cimagf(buffer[i]) * SHRT_MAX;
    }

    write(1,output,sizeof(output));
  }
  exit(0);
}
