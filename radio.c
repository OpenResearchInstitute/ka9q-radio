// $Id: radio.c,v 1.35 2017/07/02 04:29:56 karn Exp karn $
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


extern int Ctl_fd;
struct demod Demod;

//const float Headroom = .316227766; // sqrt(0.10) = -10 dB
const float Headroom = 0.1778; // -15 dB


// Get true first LO frequency
const double get_first_LO(const struct demod *demod){
  assert(demod != NULL);
  return demod->first_LO * (1 + demod->calibrate);  // True frequency, as adjusted
}


// Return current frequency of carrier frequency at current first IF
// If sweeping, return second LO freq at "offset" samples ahead of current sample
const double get_second_LO(const struct demod *demod,const int offset){
  assert(demod != NULL);

  if(cimag(demod->second_LO_phase_accel) != 0){
    // sweeping, get instantaneous frequency
    return M_1_2PI * demod->samprate
      * (offset * carg(demod->second_LO_phase_accel) + carg(demod->second_LO_phase_step));
  } else
    return demod->second_LO;
}

// Set frequency with optional front end tuning
double set_freq(struct demod *demod,const double f,const int force){
  assert(demod != NULL);

  double change = f - get_freq(demod);

  // See if we can change only LO2
  // Note lo2 is the negative of the IF
  double lo2 = get_second_LO(demod,0) - change;
  // If the new LO2 is out of range, or if we're forced, recenter LO2
  // and retune LO1
  if(force || !LO2_in_range(demod,lo2,1)){
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
  set_second_LO(demod,lo2);
  return f;
}

// Return current frequency, including effects of any sweep & dial offset
const double get_freq(const struct demod *demod){
  assert(demod != NULL);
  return get_first_LO(demod) - get_second_LO(demod,0) + demod->dial_offset;
}

// Set tuner LO
// Note: single precision floating point is not accurate enough at VHF and above
// demod->first_LO isn't updated here, but by the
// incoming status frames so it don't change right away
double set_first_LO(struct demod *demod,const double first_LO,const int force){
  assert(demod != NULL);

  if(!force && first_LO == get_first_LO(demod))
    return first_LO;

  // Set tuner to integer nearest requested frequency after decalibration
  struct status requested_status;
  requested_status.frequency = round(first_LO / (1 + demod->calibrate)); // What we send to the tuner
  // No change to gain settings
  requested_status.lna_gain = 0xff;
  requested_status.mixer_gain = 0xff;
  requested_status.if_gain = 0xff;

  // Send commands to source address of last RTP packet from front end
  if(sendto(Ctl_fd,&requested_status,sizeof(requested_status),0,(struct sockaddr *)&Input_source_address,
	    sizeof(Input_source_address)) == -1)
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

// If avoid_alias is true, return 1 if specified carrier frequency is in range of LO2 given
// sampling rate, filter setting and alias region
//
// If avoid_alias is false, simply test that specified frequency is between +/- samplerate/2
const int LO2_in_range(const struct demod *demod,const double f,int avoid_alias){
  assert(demod != NULL);
  if(avoid_alias)
    return f >= demod->min_IF + max(0,demod->high)
	    && f <= demod->max_IF + min(0,demod->low);
  else
    return f >= -demod->samprate/2 && f <= demod->samprate/2;

}

// Set second local oscillator (the one in software)
// Only limit range to +/- samprate/2; the caller must avoid the alias region, e.g., with LO2_in_range()
double set_second_LO(struct demod *demod,const double second_LO){
  assert(demod != NULL);

  // When setting frequencies, assume TCXO also drives sample clock, so use same calibration
  if(second_LO < -demod->samprate/2 || second_LO > demod->samprate/2)
    return demod->second_LO; // Don't let it go out of range

  demod->second_LO = second_LO;
  demod->second_LO_phase_step = csincos(2*M_PI*second_LO/demod->samprate);
  if(demod->second_LO_phase == 0) // In case it wasn't already set
    demod->second_LO_phase = 1;

  return second_LO;
}
double set_second_LO_rate(struct demod *demod,const double second_LO_rate,const int force){
  assert(demod != NULL);

  if(!force && second_LO_rate == demod->second_LO_rate)
    return second_LO_rate;

  const double sampsq = demod->samprate * demod->samprate;
  demod->second_LO_rate = second_LO_rate;
  if(second_LO_rate == 0){
    // if stopped, store current frequency in case somebody reads it
    demod->second_LO_phase_accel = 0;
    demod->second_LO = demod->samprate * carg(demod->second_LO_phase_step) * M_1_2PI;
  } else {
    demod->second_LO_phase_accel = csincos(2*M_PI*second_LO_rate/sampsq);
  }
  return second_LO_rate;
}
int set_mode(struct demod *demod,const enum mode mode){
  assert(demod != NULL);

  demod->terminate = 1;
  pthread_join(demod->demod_thread,NULL); // Wait for it to finish
  
  demod->terminate = 0;
  demod->mode = mode;
  demod->dial_offset = Modes[mode].dial;
  demod->low = Modes[mode].low;
  demod->high = Modes[mode].high;
  // Suppress these in display unless they're used
  demod->snr = NAN;
  demod->foffset = NAN;
  demod->pdeviation = NAN;
  demod->cphase = NAN;

  double lo2 = get_second_LO(demod,0);
  // Might now be out of range because of change in filter passband
  if(!LO2_in_range(demod,lo2,1))
    set_freq(demod,get_freq(demod),1);

  switch(mode){
  case DSB:
    pthread_create(&demod->demod_thread,NULL,demod_dsb,&Demod);
    break;
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

const int get_filter(const struct demod *demod,float *low,float *high){
  assert(demod != NULL);

  if(low != NULL)
    *low = demod->low;
  if(high != NULL)
    *high = demod->high;
  return 0;
}

int set_filter(struct demod *demod,const float low,const float high){
  assert(demod != NULL);
  assert(demod->filter != NULL);
  
  int N = demod->L + demod->M - 1;

  if(high > demod->max_IF || low < demod->min_IF || high <= low)
    return -1;

  float gain;
  if(demod->filter->type == REAL || demod->filter->type == CROSS_CONJ)
    gain = M_SQRT1_2;
  else
    gain = 1; // Complex

  complex float *response = (complex float *)fftwf_alloc_complex(N);
  // posix_memalign((void **)&response,16,N*sizeof(complex float));
  memset(response,0,N*sizeof(*response));
  int n;
  for(n=N*low/demod->samprate; n <= N*high/demod->samprate; n++)
    response[(n+N)%N] = gain;
  
  window_filter(demod->L,demod->M,response,Kaiser_beta);

  // We don't do any mutual exclusion with the demod thread so
  // never let the response pointer be invalid
  complex float *tmp = demod->filter->response;
  demod->filter->response = response;
  fftwf_free(tmp);
  demod->low = low;
  demod->high = high;
  return 0;
}



int set_cal(struct demod *demod,const double cal){
  assert(demod != NULL);

  double f = get_freq(demod);
  demod->calibrate = cal;
  demod->samprate = ADC_samprate * (1 + cal);
  set_freq(demod,f,0);
  return 0;
}
const double get_cal(const struct demod *demod){
  assert(demod != NULL);

  return demod->calibrate;
}


int spindown(struct demod *demod,complex float *data,const int len){
  assert(demod != NULL);
  assert(data != NULL);

  if(demod->second_LO_phase == 0) // Make sure it's been initalized
    demod->second_LO_phase = 1;

  // Apply 2nd LO
  int n;
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
    if(first_if + max(Modes[demod->mode].high,0) >= demod->max_IF){
      // Will hit upper end
      new_first_if = demod->min_IF - min(Modes[demod->mode].low,0);
    } else if(first_if - min(Modes[demod->mode].low,0) <= demod->min_IF){
      // Will hit lower end
      new_first_if = demod->max_IF - max(Modes[demod->mode].high,0);
    }
    if(new_first_if != first_if){
      // Make the changes
      set_first_LO(demod,get_first_LO(demod) - (new_first_if - first_if),1);
      set_second_LO(demod,-new_first_if);
    }
  }
  return 0;
}
