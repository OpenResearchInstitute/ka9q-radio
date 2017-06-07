// $Id: radio.c,v 1.24 2017/06/06 10:46:05 karn Exp karn $
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
#include "audio.h"
#include "radio.h"
#include "filter.h"
#include "dsp.h"


extern int Ctl_sock;
struct demod Demod;

const float Headroom = .316227766; // sqrt(0.10) = -10 dB


// Get true first LO frequency
const double get_first_LO(const struct demod *demod){
  return demod->first_LO * (1 + demod->calibrate);  // True frequency, as adjusted
}


// Return current frequency of carrier frequency at current first IF
// If sweeping, return second LO freq at "offset" samples ahead of current sample
const double get_second_LO(const struct demod *demod,const int offset){
  if(cimag(demod->second_LO_phase_accel) != 0){
    // sweeping, get instantaneous frequency
    return get_exact_samprate(demod)
      * (offset * carg(demod->second_LO_phase_accel) + carg(demod->second_LO_phase_step)) / (2*M_PI);
  } else 
    return demod->second_LO;
}

// Set frequency with optional front end tuning
double set_freq(struct demod *demod,const double f,const int force){
  double change = f - get_freq(demod);

  // See if we can change only LO2
  // Note lo2 is the negative of the IF
  double lo2 = get_second_LO(demod,0) - change;
  // If the new LO2 is out of range, or if we're forced, recenter LO2
  // and retune LO1
  if(force || !LO2_in_range(demod,lo2)){
    if(change < 0){
      // Assume the user will keep tuning down, so put the IF in the
      // high half (lo2 in the low half)
      lo2 = -demod->samprate/4;
    } else if(change > 0){
      // Assume the user will keep tuning up
      lo2 = demod->samprate/4;
    } else {
      // If LO2 is not close to +/-samprate/4, move it there
      if(fabs(fabs(lo2) - demod->samprate/4) > 1.0){
	lo2 = copysign(demod->samprate/4,lo2);
      }
    }
    double lo1 = f + lo2 - demod->dial_offset;

    // returns actual frequency, which may be a fraction of a Hz
    // different from requested because of calibration offset and
    // the fact that the tuner can only tune in 1 Hz steps
    lo1 = set_first_LO(demod,lo1,force);
    // Adjust LO2 for actual LO1
    lo2 = lo1 - f + demod->dial_offset;
  }
  set_second_LO(demod,lo2,force);
  return f;
}

// Return current frequency, including effects of any sweep & dial offset
const double get_freq(const struct demod *demod){
  return get_first_LO(demod) - get_second_LO(demod,0)
    + demod->dial_offset;
}

// Set tuner LO
// Note: single precision floating point is not accurate enough at VHF and above
// demod->first_LO isn't updated here, but by the
// incoming status frames so it don't change right away
double set_first_LO(struct demod *demod,const double first_LO,const int force){
  struct status requested_status;

  if(!force && first_LO == get_first_LO(demod))
    return first_LO;

  // Set tuner to integer nearest requested frequency after decalibration
  requested_status.frequency = round(first_LO / (1 + demod->calibrate)); // What we send to the tuner
  // No change to gain settings
  requested_status.lna_gain = 0xff;
  requested_status.mixer_gain = 0xff;
  requested_status.if_gain = 0xff;    

  // Send commands to source address of last RTP packet from front end
  if(sendto(Ctl_sock,&requested_status,sizeof(requested_status),0,&Rtp_source_address,
	    sizeof(Rtp_source_address)) == -1)
    perror("sendto control socket");

  // Return new true frequency
  // Wait for it to actually change before we retune the second LO
  // Adjust LO2 for any fractional Hz error in LO1
  // (What if we're reading from a recording?)
  int i;
  for(i=0;i<10;i++){
    if(demod->first_LO == requested_status.frequency)
      break;
    usleep(50000);
  }
  return requested_status.frequency * (1 + demod->calibrate);
}

// Return 1 if specified carrier frequency is in range of LO2 given
// sampling rate and filter setting
const int LO2_in_range(const struct demod *demod,const double f){
  if( f < -demod->samprate/2 + max(0,demod->high)
      || f > demod->samprate/2 + min(0,demod->low))
    return 0;
  else
    return 1;
}


double set_second_LO(struct demod *demod,const double second_LO,const int force){
  // When setting frequencies, assume TCXO also drives sample clock, so use same calibration
  if(!force && second_LO == demod->second_LO)
    return second_LO;
  
  // Don't allow a frequency that puts the passband past the nyquist rate
  // Exception: if we're already in the forbidden region, allow a change
  // that will move us toward the exit so we don't get stuck, e.g.,
  // after a filter change
  if(((second_LO < -demod->samprate/2 + max(0,demod->high))
      && (second_LO < demod->second_LO))
     || ((second_LO > demod->samprate/2 + min(0,demod->low))
      && (second_LO > demod->second_LO)))
    return demod->second_LO; // Don't let it go out of range

  demod->second_LO = second_LO;
  demod->second_LO_phase_step = csincos(2*M_PI*second_LO/get_exact_samprate(demod));
  if(demod->second_LO_phase == 0) // In case it wasn't already set
    demod->second_LO_phase = 1;

  return second_LO;
}
double set_second_LO_rate(struct demod *demod,const double second_LO_rate,const int force){
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
int set_mode(struct demod *demod,const enum mode mode){

  pthread_cancel(demod->demod_thread); // what if it's not running?
  void *retval; // Ignored
  pthread_join(demod->demod_thread,&retval); // Wait for it to finish
  
  demod->mode = mode;
  demod->dial_offset = Modes[mode].dial;
  demod->low = Modes[mode].low;
  demod->high = Modes[mode].high;

  double lo2 = get_second_LO(demod,0);
  if(!LO2_in_range(demod,lo2))
    set_freq(demod,get_freq(demod),1);

  switch(mode){
  case NFM:
  case FM:
    demod->output = Audio_mono_sock;
    pthread_create(&demod->demod_thread,NULL,demod_fm,&Demod);
    break;
  case USB:
  case LSB:
  case CWU:
  case CWL:
    demod->output = Audio_mono_sock;    
    pthread_create(&demod->demod_thread,NULL,demod_ssb,&Demod);
    break;
  case CAM:
    demod->output = Audio_mono_sock;
    pthread_create(&demod->demod_thread,NULL,demod_cam,&Demod);
    break;
  case AM:
    demod->output = Audio_mono_sock;
    pthread_create(&demod->demod_thread,NULL,demod_am,&Demod);
    break;
  case IQ:
  case ISB:
    demod->output = Audio_stereo_sock;
    pthread_create(&demod->demod_thread,NULL,demod_iq,&Demod);
    break;
  case WFM:
    break;
  }
  return 0;
}      

const int get_filter(const struct demod *demod,float *low,float *high){
  if(low != NULL)
    *low = demod->low;
  if(high != NULL)
    *high = demod->high;
  return 0;
}

int set_filter(struct demod *demod,const float low,const float high){
  float gain;
  int n;
  int N = demod->L + demod->M - 1;

  if(high > demod->samprate/2 || low < -demod->samprate/2 || high <= low)
    return -1;

  if(demod->filter->type == REAL || demod->filter->type == CROSS_CONJ)
    gain = M_SQRT1_2;
  else
    gain = 1; // Complex

  complex float *response = (complex float *)fftwf_alloc_complex(N);
  // posix_memalign((void **)&response,16,N*sizeof(complex float));
  memset(response,0,N*sizeof(*response));
  for(n=N*low/demod->samprate; n <= N*high/demod->samprate; n++)
    response[(n+N)%N] = gain;
  
  window_filter(demod->L,demod->M,response,Kaiser_beta);

  // We don't do any mutual exclusion with the demod thread so
  // never let the response pointer be invalid
  complex float *tmp;
  tmp = demod->filter->response;
  demod->filter->response = response;
  fftwf_free(tmp);
  demod->low = low;
  demod->high = high;
  return 0;
}



int set_cal(struct demod *demod,double cal){
  double f = get_freq(demod);
  demod->calibrate = cal;
  set_freq(demod,f,0);
  return 0;
}
const double get_cal(const struct demod *demod){
  return demod->calibrate;
}


int spindown(struct demod *demod,complex float *data,const int len){
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

const double get_exact_samprate(const struct demod *demod){
  return demod->samprate * (1 + demod->calibrate);
}
