// $Id: display.c,v 1.4 2016/10/14 07:37:13 karn Exp karn $
// Thread to display internal state of 'radio' command on command line 
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#undef I
#include <stdio.h>
#include <stdlib.h>

#include "radio.h"
#include "audio.h"
#include "dsp.h"

void *display(void *arg){
  fprintf(stderr,"Mode        Frequency         First LO     First IF  Offset    IF1    IF2  AFgain  FM SNR  D/A Under\n");
  fprintf(stderr,"                   Hz               Hz           Hz      Hz     dB     dB      dB      dB\n");
  for(;;){
    fprintf(stderr,"%-5s%'16.2f%'17.2f%'13.2f%'8.2f%7.1f%7.1f%8.1f%8.2f%11d\r",
	    Modes[Demod.mode].name,
	    Demod.first_LO - Demod.second_LO + Modes[Demod.mode].dial,
	    Demod.first_LO,
	    -get_second_LO(0),
	    Modes[Demod.mode].dial,
	    power2dB(Demod.energy),
	    voltage2dB(Demod.amplitude),
	    voltage2dB(Demod.gain),
	    power2dB(Demod.snr),
	    Audio.underrun);
    usleep(100000); // 10 Hz update
  }
  pthread_exit(0);
}
