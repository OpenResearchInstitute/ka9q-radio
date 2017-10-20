// $Id: modulate.c,v 1.6 2017/09/19 12:58:52 karn Exp karn $ AM modulator - will eventually support other modes
// Copyright 2017, Phil Karn, KA9Q
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <limits.h>
#include <fftw3.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/resource.h>

#include "filter.h"
#include "dsp.h"
#include "radio.h"

#define BLOCKSIZE 4096

float const scale = 1./SHRT_MAX;

int Samprate = 192000;

int Verbose = 0;

int main(int argc,char *argv[]){
  // if we have root, up our priority and drop privileges
  int prio = getpriority(PRIO_PROCESS,0);
  prio = setpriority(PRIO_PROCESS,0,prio - 10);

  // Quickly drop root if we have it
  // The sooner we do this, the fewer options there are for abuse
  seteuid(getuid());


  int c;

  // Set defaults
  double frequency = 48000;
  double amplitude = -20;
  double sweep = 0;

  while((c = getopt(argc,argv,"f:a:s:r:v")) != EOF){
    switch(c){
    case 'v':
      Verbose++;
      break;
    case 'r':
      Samprate = strtol(optarg,NULL,0);
      break;
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
  if(Verbose){
    fprintf(stderr,"AM modulation on %.1f Hz IF, swept %.1f Hz/s, amplitude %5.1f dBFS, filter blocksize %'d\n",
	    frequency,sweep,amplitude,BLOCKSIZE);
  }

  frequency *= 2*M_PI/Samprate;       // radians/sample
  amplitude = pow(10.,amplitude/20.); // Convert to amplitude ratio
  sweep *= 2*M_PI / ((double)Samprate*Samprate);  // radians/sample

  complex double phase_accel = csincos(sweep);
  complex double phase_step = csincos(frequency);
  complex double phase = 1;  
  int const L = BLOCKSIZE;
  int const M = BLOCKSIZE + 1;
  int const N = L + M - 1;

  complex float * const response = fftwf_alloc_complex(N);
  {
    float gain = 4./N; // Compensate for FFT/IFFT scaling and 4x upsampling
    for(int i=0;i<N;i++){
      float f;
      f = Samprate * ((float)i/N);
      if(f > Samprate/2)
	f -= Samprate;
      if(f >= -8000 && f <= +8000)
	response[i] = gain;
    }
  }
  window_filter(L,M,response,3.0);
  struct filter * const filter = create_filter(L,M,response,1,REAL,COMPLEX);

  while(1){
    int16_t samp[L/4];
    if(pipefill(0,samp,sizeof(samp)) <= 0)
      break;
    // Filter will upsample by 4x
    for(int j=0,i=0;i<L;){
      filter->input.r[i++] = samp[j++] * scale;
      filter->input.r[i++] = 0;
      filter->input.r[i++] = 0;
      filter->input.r[i++] = 0;      
    }
    // Form baseband signal (analytic for SSB, pure real for AM/DSB)
    execute_filter(filter);
    
    // Add AM carrier
    for(int i=0;i<L;i++)
      filter->output.c[i] += 1;

    // Spin up to chosen carrier frequency
    for(int i=0;i<L;i++){
      filter->output.c[i] *= phase * amplitude;
      phase *= phase_step;
      phase_step *= phase_accel;
    }
    phase /= cabs(phase);
    phase_step /= cabs(phase_step);
    int16_t output[2*L];
    for(int i=0;i<L;i++){
      output[2*i] = crealf(filter->output.c[i]) * SHRT_MAX;
      output[2*i+1] = cimagf(filter->output.c[i]) * SHRT_MAX;
    }
    write(1,output,sizeof(output));
  }
  delete_filter(filter);
  exit(0);
}
