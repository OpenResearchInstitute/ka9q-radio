// $Id: radio.c,v 1.2 2016/10/13 23:57:15 karn Exp karn $
// Lower part of radio program - control LOs, set frequency/mode, etc
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#undef I

#include "command.h"
#include "radio.h"
#include "filter.h"
#include "dsp.h"
#include "audio.h"


struct demod Demod;
extern int Front_end_ctl;
double get_exact_samprate();


// Return current frequency of carrier frequency at current first IF, including sweep
double get_second_LO(){
  if(Demod.second_LO_phase_accel != 0){
    // sweeping, get instantaneous frequency
    Demod.second_LO = get_exact_samprate() * carg(Demod.second_LO_phase_step) / (2*M_PI);
  }
  return Demod.second_LO;
}

// Return current carrier frequency, including effects of any sweep
double get_freq(){
  return Demod.first_LO - get_second_LO();
}

// Set either or both LOs as needed to tune the specified radio frequency to zero audio frequency
// Actually it goes to the offset specified in the mode table; e.g. +/- 750 Hz for the CW modes
// Note: single precision floating point is not accurate enough at VHF and above
double set_first_LO(double first_LO,int force){
  struct command command;

  if(!force && first_LO == Demod.first_LO)
    return first_LO;

  memset(&command,0,sizeof(command));
  command.cmd = SETSTATE;
  command.first_LO = round(first_LO / (1 + Demod.calibrate)); // What we send to the tuner
  Demod.first_LO = command.first_LO * (1 + Demod.calibrate);  // True frequency, as adjusted
  Demod.first_LO_err = Demod.first_LO - first_LO; // fractional Hz between actual and desired
  write(Front_end_ctl,&command,sizeof(command));
  return Demod.first_LO;
}
double set_second_LO(double second_LO,int force){
  // When setting frequencies, assume TCXO also drives sample clock, so use same calibration
  if(!force && second_LO + Demod.first_LO_err == Demod.second_LO)
    return second_LO;
  
  Demod.second_LO = second_LO + Demod.first_LO_err;
  Demod.second_LO_phase_step = csincos(2*M_PI*Demod.second_LO/get_exact_samprate());
  if(Demod.second_LO_phase == 0) // In case it wasn't already set
    Demod.second_LO_phase = 1;
  return second_LO;
}
double set_second_LO_rate(double second_LO_rate,int force){
  double samprate,sampsq;

  if(!force && second_LO_rate == Demod.second_LO_rate)
    return second_LO_rate;

  samprate = get_exact_samprate(); // calibrated sample rate
  sampsq = samprate * samprate;
  Demod.second_LO_rate = second_LO_rate;
  if(second_LO_rate == 0){
    // if stopped, store current frequency in case somebody reads it
    Demod.second_LO_phase_accel = 0;
    Demod.second_LO = get_exact_samprate() * carg(Demod.second_LO_phase_step)/(2*M_PI);
  } else {
    Demod.second_LO_phase_accel = csincos(2*M_PI*second_LO_rate/sampsq);
  }
  return second_LO_rate;
}
int set_mode(enum mode mode){
  const int N = Demod.L + Demod.M - 1;
  int n;

  if(mode == Demod.mode)
    return 0;
  
  Demod.mode = mode;
  Demod.low = N*Modes[mode].low/Demod.samprate;
  Demod.high = N*Modes[mode].high/Demod.samprate;
  if(Demod.low > Demod.high){
    int t;
    t = Demod.low;
    Demod.low = Demod.high;
    Demod.high = t;
  }
  Demod.hangmax = 1.1 * (Demod.samprate/Demod.L); // 1.1 second hang before gain increase
  Demod.agcratio = dB2voltage(6 * ((float)Demod.L/Demod.samprate)); // 6 dB/sec
  Demod.hangtime = 0;
  Demod.fmstate = 0;
  if(Demod.filter != NULL){
    delete_filter(Demod.filter);
    Demod.response = NULL;
    Demod.filter = NULL;
  }
  
  // Set up pre-demodulation filter
  //  Could say 'Demod.response = fftwf_alloc_complex(N);' if we include fftw3.h
  posix_memalign((void **)&Demod.response,16,N*sizeof(complex float));
  memset(Demod.response,0,N*sizeof(*Demod.response));
  for(n=Demod.low; n <= Demod.high; n++)
    Demod.response[(n+N)%N] = 1;
  
  window_filter(Demod.L,Demod.M,Demod.response,Kaiser_beta);
  Demod.decimate = Demod.samprate / Audio.samprate;
  
  switch(mode){
  case USB:
  case LSB:
  case CWU:
  case CWL:
    Demod.filter = create_filter(Demod.L,Demod.M,Demod.response,Demod.decimate,REAL);
    break;
  case ISB:
    Demod.filter = create_filter(Demod.L,Demod.M,Demod.response,Demod.decimate,CROSS_CONJ);
    break;
  default:
    Demod.filter = create_filter(Demod.L,Demod.M,Demod.response,Demod.decimate,COMPLEX);
    break;
  }
  // Used by FM only; automatically adjusted by AGC in linear modes
  if(mode == FM)
    Demod.gain = N / (Demod.decimate * abs(Demod.low - Demod.high));
  else
    Demod.gain = dB2voltage(70.); // 70 dB
  //  audio_change_parms(Demod.samprate/Demod.decimate,Modes[mode].channels,Demod.L);
  audio_change_parms(Demod.samprate/Demod.decimate,2,Demod.L);  
  return 0;
}      
int set_cal(double cal){
  Demod.calibrate = cal;
  set_first_LO(Demod.first_LO,1);
  set_second_LO(Demod.second_LO,1);
  set_second_LO_rate(Demod.second_LO_rate,1);
  return 0;
}


int spindown(complex float *data,int len){
  int n;

  if(Demod.second_LO_phase == 0) // Make sure it's been initalized
    Demod.second_LO_phase = 1;

  // Apply 2nd LO
  for(n=0; n < len; n++){
    data[n] *= Demod.second_LO_phase;
    Demod.second_LO_phase *= Demod.second_LO_phase_step;
    if(Demod.second_LO_phase_accel != 0)
      Demod.second_LO_phase_step *= Demod.second_LO_phase_accel; // Frequency sweep
  }
  // Renormalize to guard against accumulated roundoff error
  Demod.second_LO_phase /= cabs(Demod.second_LO_phase);
  if(Demod.second_LO_phase_accel != 0)
    Demod.second_LO_phase_step /= cabs(Demod.second_LO_phase_step);

  return 0;
}


int ssb_agc(){
  if(Demod.gain * Demod.amplitude > 0.3){ // Target to -10 dBFS
    // New signal peak: decrease gain and inhibit re-increase for a while
    Demod.gain = 0.3 / Demod.amplitude;
    Demod.gain = max(Demod.gain,1.0); // 0 dB floor in case of a big peak
    Demod.hangtime = Demod.hangmax;
  } else {
    if(Demod.hangtime !=0){
      Demod.hangtime--;
    } else {
      // OK to increase gain; should enforce a limit
      Demod.gain *= Demod.agcratio;
    }
  }
  return 0;
}



double get_exact_samprate(){
  return Demod.samprate * (1 + Demod.calibrate);
}
