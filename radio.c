// $Id: radio.c,v 1.9 2017/05/25 11:15:25 karn Exp karn $
// Lower part of radio program - control LOs, set frequency/mode, etc
#define _GNU_SOURCE 1
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>
#undef I
#include <sys/socket.h>
#include <netinet/in.h>


#include "command.h"
#include "radio.h"
#include "filter.h"
#include "dsp.h"
#include "audio.h"


extern int ctl_sock;
struct demod Demod;
double get_exact_samprate();

const float Headroom = .316227766; // sqrt(0.10) = -10 dB


// Get true first LO frequency
double get_first_LO(){
  return Demod.first_LO * (1 + Demod.calibrate);  // True frequency, as adjusted
}


// Return current frequency of carrier frequency at current first IF
// If sweeping, return second LO freq at "offset" samples ahead of current sample
double get_second_LO(int offset){
  if(cimag(Demod.second_LO_phase_accel) != 0){
    // sweeping, get instantaneous frequency
    Demod.second_LO = get_exact_samprate()
      * (offset * carg(Demod.second_LO_phase_accel) + carg(Demod.second_LO_phase_step)) / (2*M_PI);
  }
  return Demod.second_LO;
}

// Return current carrier frequency, including effects of any sweep
double get_freq(){
  return get_first_LO() - get_second_LO(0);
}

// Set either or both LOs as needed to tune the specified radio frequency to zero audio frequency
// Actually it goes to the offset specified in the mode table; e.g. +/- 750 Hz for the CW modes
// Note: single precision floating point is not accurate enough at VHF and above
int set_first_LO(double first_LO,int force){
  struct status requested_status;
  struct sockaddr_in6 FE_address;

  if(!force && first_LO == get_first_LO())
    return 0;

  // Change only tuner frequency
  requested_status.frequency = round(first_LO / (1 + Demod.calibrate)); // What we send to the tuner
  requested_status.lna_gain = 0xff;
  requested_status.mixer_gain = 0xff;
  requested_status.if_gain = 0xff;    

  // Send commands to source address of last RTP packet from front end
  FE_address = rtp_address;
  FE_address.sin6_port = htons(4160); // make this better!
  if(sendto(ctl_sock,&requested_status,sizeof(requested_status),0,&FE_address,sizeof(FE_address)) == -1)
      perror("sendto control socket");
  return 0;
}
double set_second_LO(double second_LO,int force){
  // When setting frequencies, assume TCXO also drives sample clock, so use same calibration
  if(!force && second_LO == Demod.second_LO)
    return second_LO;
  
  Demod.second_LO = second_LO;
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

  void *demod_fm(void *);
  void *demod_ssb(void *);
  void *demod_iq(void *);
  void *demod_cam(void *);
  void *demod_am(void *);  

#if 0
  if(mode == Demod.mode)
    return 0;
#endif
  
  Demod.mode = mode;
  Demod.samprate = 192000;
  Demod.L = 4096;
  Demod.M = 4097;
  
  pthread_cancel(Demod.demod_thread); // what if it's not running?
  void *retval;
  pthread_join(Demod.demod_thread,&retval); // Wait for it to finish
  
  switch(mode){
  case NFM:
  case FM:
    pthread_create(&Demod.demod_thread,NULL,demod_fm,NULL);
    break;
  case USB:
  case LSB:
  case CWU:
  case CWL:
    pthread_create(&Demod.demod_thread,NULL,demod_ssb,NULL);
    break;
  case CAM:
    pthread_create(&Demod.demod_thread,NULL,demod_cam,NULL);
    break;
  case AM:
    pthread_create(&Demod.demod_thread,NULL,demod_am,NULL);
    break;
  case IQ:
  case ISB:
    pthread_create(&Demod.demod_thread,NULL,demod_iq,NULL);
    break;
  case WFM:
    break;
  }
  return 0;
}      
int set_cal(double cal){
  Demod.calibrate = cal;
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

  if(cimag(Demod.second_LO_phase_accel) != 0){
    // We're sweeping, so ensure we won't run the passband past the edges of the first IF bandwidth
    double first_if = -get_second_LO(Demod.filter->blocksize_in);  // first IF at end of *next* sample block
    double new_first_if = first_if;
    if(first_if + max(Modes[Demod.mode].high,0) >= Demod.samprate/2){
      // Will hit upper end
      new_first_if = -Demod.samprate/2 - min(Modes[Demod.mode].low,0);
    } else if(first_if - min(Modes[Demod.mode].low,0) <= -Demod.samprate/2){
      // Will hit lower end
      new_first_if = Demod.samprate/2 - max(Modes[Demod.mode].high,0);
    }
    if(new_first_if != first_if){
      // Make the changes
      set_first_LO(get_first_LO() - (new_first_if - first_if),1);
      set_second_LO(-new_first_if,1);
    }
  }
  return 0;
}


int ssb_agc(){
  if(Demod.gain * Demod.amplitude > Headroom){ // Target to about -10 dBFS
    // New signal peak: decrease gain and inhibit re-increase for a while
    Demod.gain = Headroom / Demod.amplitude;
    Demod.hangtime = Demod.hangmax;
  } else {
    // Not a new peak, but the AGC is still hanging at the last peak
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
