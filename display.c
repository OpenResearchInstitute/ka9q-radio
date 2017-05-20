// $Id: display.c,v 1.5 2017/05/11 10:32:06 karn Exp karn $
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
  fprintf(stderr,"Mode        Frequency         First LO     First IF  Offset    IF1    IF2  AFgain     SNR  D/A Under    DCI    DCQ\n");
  fprintf(stderr,"                   Hz               Hz           Hz      Hz     dB     dB      dB      dB\n");
  for(;;){
    fprintf(stderr,"%-5s%'16.2f%'17.2f%'13.2f%'8.2f%7.1f%7.1f%8.1f%8.2f%11d%7.1f%7.1f\r",
	    Modes[Demod.mode].name,
	    get_first_LO() - get_second_LO(0) + Modes[Demod.mode].dial,
	    get_first_LO(),
	    -get_second_LO(0),
	    Modes[Demod.mode].dial,
	    power2dB(crealf(Demod.power) + cimagf(Demod.power)),
	    voltage2dB(Demod.amplitude),
	    voltage2dB(Demod.gain),
	    power2dB(Demod.snr),
	    Audio.underrun,
	    crealf(Demod.DC_offset),
	    cimagf(Demod.DC_offset));
    usleep(100000); // 10 Hz update
  }
  pthread_exit(0);
}
