// $Id: radio.c,v 1.14 2017/05/31 22:27:14 karn Exp karn $
// Lower part of radio program - control LOs, set frequency/mode, etc
#define _GNU_SOURCE 1
#include <assert.h>
#include <unistd.h>
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


extern int Ctl_sock;
struct demod Demod;

const float Headroom = .316227766; // sqrt(0.10) = -10 dB


// Get true first LO frequency
double get_first_LO(struct demod *demod){
  return demod->first_LO * (1 + demod->calibrate);  // True frequency, as adjusted
}


// Return current frequency of carrier frequency at current first IF
// If sweeping, return second LO freq at "offset" samples ahead of current sample
double get_second_LO(struct demod *demod,int offset){
  if(cimag(demod->second_LO_phase_accel) != 0){
    // sweeping, get instantaneous frequency
    demod->second_LO = get_exact_samprate(demod)
      * (offset * carg(demod->second_LO_phase_accel) + carg(demod->second_LO_phase_step)) / (2*M_PI);
  }
  return demod->second_LO;
}

// Set frequency with optional front end tuning
double set_freq(struct demod *demod,double f,int force){
  double change = f - get_freq(demod);
  double lo1 = get_first_LO(demod);
  double lo2 = get_second_LO(demod,0) - change;

  if(force
     || -lo2 >= demod->samprate/2 - max(0,Modes[demod->mode].high)
     || -lo2 <= -demod->samprate/2 - min(0,Modes[demod->mode].low)){
    if(fabs(change) >= demod->samprate/2 || change < 0)
      lo2 = -48000;
    else
      lo2 = +48000;
  }
  lo1 = f + lo2 - demod->dial_offset;
  lo1 = set_first_LO(demod,lo1,force);
  int i;
  for(i=0;i<10;i++){
    if(get_first_LO(demod) == lo1)
      break;
    usleep(50000);
  }
  lo2 = lo1 - f + demod->dial_offset;
  set_second_LO(demod,lo2,force);
  return f;
}

// Return current carrier frequency, including effects of any sweep
double get_freq(struct demod *demod){
  return get_first_LO(demod) - get_second_LO(demod,0)
    + demod->dial_offset;
}

// Set either or both LOs as needed to tune the specified radio frequency to zero audio frequency
// Actually it goes to the offset specified in the mode table; e.g. +/- 750 Hz for the CW modes
// Note: single precision floating point is not accurate enough at VHF and above
double set_first_LO(struct demod *demod,double first_LO,int force){
  struct status requested_status;
  struct sockaddr_in6 FE_address;

  if(!force && first_LO == get_first_LO(demod))
    return get_first_LO(demod);

  // Change only tuner frequency
  requested_status.frequency = round(first_LO / (1 + demod->calibrate)); // What we send to the tuner
  requested_status.lna_gain = 0xff;
  requested_status.mixer_gain = 0xff;
  requested_status.if_gain = 0xff;    

  // Send commands to source address of last RTP packet from front end
  FE_address = rtp_address;
  FE_address.sin6_port = htons(4160); // make this better!
  if(sendto(Ctl_sock,&requested_status,sizeof(requested_status),0,&FE_address,sizeof(FE_address)) == -1)
      perror("sendto control socket");
  return requested_status.frequency * (1 + demod->calibrate);
}
double set_second_LO(struct demod *demod,double second_LO,int force){
  // When setting frequencies, assume TCXO also drives sample clock, so use same calibration
  if(!force && second_LO == demod->second_LO)
    return second_LO;
  
  demod->second_LO = second_LO;
  demod->second_LO_phase_step = csincos(2*M_PI*demod->second_LO/get_exact_samprate(demod));
  if(demod->second_LO_phase == 0) // In case it wasn't already set
    demod->second_LO_phase = 1;
  return second_LO;
}
double set_second_LO_rate(struct demod *demod,double second_LO_rate,int force){
  double samprate,sampsq;

  if(!force && second_LO_rate == demod->second_LO_rate)
    return second_LO_rate;

  samprate = get_exact_samprate(demod); // calibrated sample rate
  sampsq = samprate * samprate;
  demod->second_LO_rate = second_LO_rate;
  if(second_LO_rate == 0){
    // if stopped, store current frequency in case somebody reads it
    demod->second_LO_phase_accel = 0;
    demod->second_LO = get_exact_samprate(demod) * carg(demod->second_LO_phase_step)/(2*M_PI);
  } else {
    demod->second_LO_phase_accel = csincos(2*M_PI*second_LO_rate/sampsq);
  }
  return second_LO_rate;
}
int set_mode(struct demod *demod,enum mode mode){


#if 0
  if(mode == demod->mode)
    return 0;
#endif
  
  demod->mode = mode;
  demod->samprate = 192000;
  demod->L = 4096;
  demod->M = 4097;
  demod->dial_offset = Modes[mode].dial;
  
  pthread_cancel(demod->demod_thread); // what if it's not running?
  void *retval;
  pthread_join(demod->demod_thread,&retval); // Wait for it to finish
  
  switch(mode){
  case NFM:
  case FM:
    pthread_create(&demod->demod_thread,NULL,demod_fm,&Demod);
    break;
  case USB:
  case LSB:
  case CWU:
  case CWL:
    pthread_create(&demod->demod_thread,NULL,demod_ssb,&Demod);
    break;
  case CAM:
    pthread_create(&demod->demod_thread,NULL,demod_cam,&Demod);
    break;
  case AM:
    pthread_create(&demod->demod_thread,NULL,demod_am,&Demod);
    break;
  case IQ:
  case ISB:
    pthread_create(&demod->demod_thread,NULL,demod_iq,&Demod);
    break;
  case WFM:
    break;
  }
  return 0;
}      
int set_cal(struct demod *demod,double cal){
  double f = get_freq(demod);
  demod->calibrate = cal;
  set_freq(demod,f,0);
  return 0;
}
double get_cal(struct demod *demod){
  return demod->calibrate;
}



int spindown(struct demod *demod,complex float *data,int len){
  int n;

  if(demod->second_LO_phase == 0) // Make sure it's been initalized
    demod->second_LO_phase = 1;

  // Apply 2nd LO
  for(n=0; n < len; n++){
    data[n] *= demod->second_LO_phase;
    demod->second_LO_phase *= demod->second_LO_phase_step;
    if(demod->second_LO_phase_accel != 0)
      demod->second_LO_phase_step *= demod->second_LO_phase_accel; // Frequency sweep
  }
  // Renormalize to guard against accumulated roundoff error
  demod->second_LO_phase /= cabs(demod->second_LO_phase);
  if(demod->second_LO_phase_accel != 0)
    demod->second_LO_phase_step /= cabs(demod->second_LO_phase_step);

  if(cimag(demod->second_LO_phase_accel) != 0){
    // We're sweeping, so ensure we won't run the passband past the edges of the first IF bandwidth
    double first_if = -get_second_LO(demod,demod->filter->blocksize_in);  // first IF at end of *next* sample block
    double new_first_if = first_if;
    if(first_if + max(Modes[demod->mode].high,0) >= demod->samprate/2){
      // Will hit upper end
      new_first_if = -demod->samprate/2 - min(Modes[demod->mode].low,0);
    } else if(first_if - min(Modes[demod->mode].low,0) <= -demod->samprate/2){
      // Will hit lower end
      new_first_if = demod->samprate/2 - max(Modes[demod->mode].high,0);
    }
    if(new_first_if != first_if){
      // Make the changes
      set_first_LO(demod,get_first_LO(demod) - (new_first_if - first_if),1);
      set_second_LO(demod,-new_first_if,1);
    }
  }
  return 0;
}

double get_exact_samprate(struct demod *demod){
  return demod->samprate * (1 + demod->calibrate);
}
