// $Id: main.c,v 1.3 2016/10/14 01:22:42 karn Exp karn $
// Read complex float samples from stdin (e.g., from funcube.c)
// downconvert, filter and demodulate
// Take commands from UDP socket
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#undef I
#include <stdio.h>
#include <stdlib.h>
#include <locale.h>
#include <signal.h>

#include "radio.h"
#include "filter.h"
#include "dsp.h"
#include "audio.h"
#include "command.h"

void closedown(int a);

pthread_t Display_thread;

int Nthreads = 1;
int ADC_samprate = 192000;
int DAC_samprate = 48000;
char *Tuner_host = "localhost";

int Quiet;


int main(int argc,char *argv[]){
  int c,N;
  char *locale;
  double rf = 162400000;
  struct command command;

  Quiet = 0;
  memset(&command,0,sizeof(command));
  locale = getenv("LANG");
  setlocale(LC_ALL,locale);

  fprintf(stderr,"General coverage receiver for the Funcube Pro and Pro+\n");
  fprintf(stderr,"Copyright 2016 by Phil Karn, KA9Q; may be used under the terms of the GNU General Public License\n");
  fprintf(stderr,"Compiled %s on %s\n",__TIME__,__DATE__);

  locale = getenv("LANG");

  Audio.name = "sysdefault";
  Demod.L = 4096;      // Number of samples in buffer
  Demod.M = (4096+1);  // Length of filter impulse response

  command.cmd = SETSTATE;
  command.second_LO = -48000;
  command.first_LO = rf + command.second_LO;
  command.second_LO_rate = 0;
  command.mode = FM;
  
  // Defaults
  while((c = getopt(argc,argv,"qb:m:l:f:d:S:L:M:x:h:r:t:c:eT:")) != EOF){
    int i;

    switch(c){
    case 'q':
      Quiet++; // Suppress display
      break;
    case 'b':
      Kaiser_beta = atof(optarg);
      break;
    case 'e':
      Audio.echo = 1; // Echo sound to standard output
      break;
    case 'c':
      command.calibrate = atof(optarg)*1e-6;
      break;
    case 'h':
      Demod.min_IF = atof(optarg);
      break;
    case 'x':
      Demod.max_IF = atof(optarg);
      break;
    case 'L':
      Demod.L = atoi(optarg);
      break;
    case 'M':
      Demod.M = atoi(optarg);
      break;
    case 'r':
      DAC_samprate = atof(optarg);
      break;
    case 'f':
      rf = atof(optarg);
      break;
    case 'm':
      for(i = 1; i < Nmodes;i++){
	if(strcasecmp(optarg,Modes[i].name) == 0){
	  command.mode = Modes[i].mode;
	  break;
	}
      }
      break;
    case 'l':
      locale = optarg;
      break;
    case 'S':
      Audio.name = optarg;
      break;
    case 'T':
      Tuner_host = optarg;
      break;
    case 't':
      Nthreads = atoi(optarg);
      fftwf_init_threads();
      fftwf_plan_with_nthreads(Nthreads);
      fprintf(stderr,"Using %d threads for FFTs\n",Nthreads);
      break;
    default:
      fprintf(stderr,"Usage: %s [-r DAC sample rate] [-f freq] [-m mode] [-l locale] [-S sound output device] [-L samplepoints] [-M impulsepoints] [-h zeroIFhole] [-x maxIF]\n",argv[0]);
      fprintf(stderr,"Default: %s -r %u -f %.1lf -m %s -l %s -S %s -L %d -M %d -h %.1f -x %.1f\n",
	      argv[0],DAC_samprate,rf,Modes[command.mode].name,locale,Audio.name,Demod.L,Demod.M,Demod.max_IF,Demod.min_IF);
      break;
    }
  }
  setlocale(LC_ALL,locale);


  Demod.samprate = 192000; // clean this up
  Audio.samprate = DAC_samprate;
  // Verify decimation ratio
  if((Demod.samprate % Audio.samprate) != 0)
    fprintf(stderr,"Warning: A/D rate %'u is not integer multiple of D/A rate %'u; decimation will probably fail\n",
	    Demod.samprate,Audio.samprate);
  Demod.decimate = Demod.samprate / Audio.samprate;
  
  N = Demod.L + Demod.M - 1;
  if((N % Demod.decimate) != 0)
    fprintf(stderr,"Warning: FFT size %'u is not divisible by decimation ratio %d\n",N,Demod.decimate);

  if((Demod.M - 1) % Demod.decimate != 0)
    fprintf(stderr,"Warning: Filter length %'u - 1 is not divisible by decimation ratio %d\n",Demod.M,Demod.decimate);

  // Must do this before first filter is created with set_mode(), otherwise a segfault can occur
  fftwf_import_system_wisdom();

  if(Demod.max_IF == 0)
    Demod.max_IF = Demod.samprate/2;
  if(Demod.max_IF > (double)Demod.samprate/2)
    Demod.max_IF = (double)Demod.samprate/2;
  
  fprintf(stderr,"A/D sample rate %'d, D/A sample rate %'d, decimation ratio %'d\n",
	  Demod.samprate,Audio.samprate,Demod.decimate);
  fprintf(stderr,"block size: %'d complex samples (%'.1f ms @ %'u S/s)\n",
	  Demod.L,1000.*Demod.L/Demod.samprate,Demod.samprate);
  fprintf(stderr,"Kaiser beta %.1lf, impulse response: %'d complex samples (%'.1f ms @ %'u S/s) bin size %.1f Hz\n",
	  Kaiser_beta,Demod.M,1000.*Demod.M/Demod.samprate,Demod.samprate,(float)Demod.samprate/N);

  if(!Quiet)
    pthread_create(&Display_thread,NULL,display,NULL);

  command.first_LO = rf + command.second_LO;
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        

  demod_loop(&command);
  if(!Quiet)
    fprintf(stderr,"radio: EOF on input\n");
  exit(0);
}

void closedown(int a){
  if(!Quiet)
    fprintf(stderr,"radio: caught signal %d: %s\n",a,strsignal(a));
  exit(1);
}
