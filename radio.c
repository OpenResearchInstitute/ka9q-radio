// $Id: radio.c,v 1.103 2018/12/02 09:41:57 karn Exp karn $
// Core of 'radio' program - control LOs, set frequency/mode, etc
// Copyright 2018, Phil Karn, KA9Q
#define _GNU_SOURCE 1
#include <assert.h>
#include <stdlib.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#if defined(linux)
#include <bsd/string.h>
#endif
#include <math.h>
#include <complex.h>
#include <fftw3.h>
#undef I
#include <netinet/in.h>

#include "misc.h"
#include "dsp.h"
#include "osc.h"
#include "radio.h"
#include "filter.h"


// SDR alias keep-out region, i.e., stay between -(samprate/2 - IF_EXCLUDE) and (samprate/2 - IF_EXCLUDE)
const float IF_EXCLUDE = 0.95; // Assume decimation filters roll off above Fs/2 * IF_EXCLUDE

// Preferred A/D sample rate; ignored by funcube but may be used by others someday
const int ADC_samprate = 192000;

// thread for first half of demodulator
// Preprocessing of samples performed for all demodulators
// Update power measurement
// Pass to input of pre-demodulation filter

float const SCALE16 = 1./SHRT_MAX; // Scale signed 16-bit int to float in range -1, +1
float const SCALE8 = 1./127;       // Scale signed 8-bit int to float in range -1, +1

void *proc_samples(void *arg){
  // gain and phase balance coefficients
  assert(arg);
  pthread_setname("procsamp");

  struct demod *demod = (struct demod *)arg;
  float complex block_energy = 0;
  int in_cnt = 0;
  struct packet *pkt = NULL;

  while(1){
    // Pull next I/Q data packet off queue
    pthread_mutex_lock(&demod->qmutex);
    while(demod->queue == NULL)
      pthread_cond_wait(&demod->qcond,&demod->qmutex);
    pkt = demod->queue;
    demod->queue = pkt->next;
    pthread_mutex_unlock(&demod->qmutex);

    int sampcount;

    switch(pkt->rtp.type){
    default: // Shut up lint
    case IQ_PT:
      sampcount = pkt->len / (2 * sizeof(signed short));
      break;
    case IQ_PT8:
      sampcount = pkt->len / (2 * sizeof(signed char));
      break;
    }
    if(pkt->rtp.ssrc != demod->input.rtp.ssrc){
      // SSRC changed; reset sample count.
      // rtp_process will reset packet count
      demod->input.samples = 0;
    }
    int time_step = rtp_process(&demod->input.rtp,&pkt->rtp,sampcount);
    if(time_step < 0 || time_step > 192000){
      // Old samples, or too big a jump; drop. Shouldn't happen if sequence number isn't old
      free(pkt); pkt = NULL;
      continue;
    } else if(time_step > 0){
      // Samples were lost. Inject enough zeroes to keep the sample count and LO phase correct
      // Arbitrary 1 sec limit just to keep things from blowing up
      // Good enough for the occasional lost packet or two
      // May upset the I/Q DC offset and channel balance estimates, but hey you can't win 'em all
      // Note: we don't use marker bits since we don't suppress silence
      demod->input.samples += time_step;
      for(int i=0;i < time_step; i++){
	demod->filter.in->input.c[in_cnt++] = 0;
	// Keep the LOs running
	(void) step_osc(&demod->second_LO);
	(void) step_osc(&demod->doppler);

	if(in_cnt == demod->filter.in->ilen){
	  // Run filter but freeze everything else?
	  execute_filter_input(demod->filter.in);
	  in_cnt = 0;
	}
      }
    }
    // Process individual samples
    signed short *sp = (signed short *)pkt->data;
    signed char *cp = (signed char *)pkt->data;
    demod->input.samples += sampcount;

    while(sampcount--){

      float samp_i,samp_q;

      switch(pkt->rtp.type){
      default: // shuts up lint
      case IQ_PT:
	samp_i = *sp++ * SCALE16;
	samp_q = *sp++ * SCALE16;
	break;
      case IQ_PT8:
	samp_i = *cp++ * SCALE8;
	samp_q = *cp++ * SCALE8;
	break;
      }
      // Scale down according to analog gain from SDR front end
      complex float samp = CMPLXF(samp_i,samp_q) * demod->gain_factor;
      block_energy += cnrmf(samp);

#if 0
      // Experimental notch filter
      if(demod->nf)
	samp = notch(demod->nf,samp);
#endif

      // Apply 2nd LO
      samp *= step_osc(&demod->second_LO);
      
      // Apply Doppler if active
      if(demod->doppler.freq != 0)
	samp *= step_osc(&demod->doppler);

      // Accumulate in filter input buffer
      demod->filter.in->input.c[in_cnt++] = samp;
      if(in_cnt == demod->filter.in->ilen){
	// Filter buffer is full, execute it
	execute_filter_input(demod->filter.in);
	block_energy *= 0.5; // Scale for two components per complex sample
	demod->if_power = block_energy / in_cnt; // Raw A/D level, without analog gain adjustment
	in_cnt = 0;
      } // Every FFT block
    } // for each sample in I/Q packet
    free(pkt); pkt = NULL;
  } // end of main loop
}

// Get true first LO frequency, with TCXO offset applied
double const get_first_LO(const struct demod * const demod){
  if(demod == NULL)
    return NAN;
	 
  return demod->status.frequency;
}


// Return second (software) local oscillator frequency
double get_second_LO(struct demod * const demod){
  if(demod == NULL)
    return NAN;
  pthread_mutex_lock(&demod->second_LO.mutex);
  double f = demod->second_LO.freq * demod->input.samprate;
  pthread_mutex_unlock(&demod->second_LO.mutex);  
  return f;
}

// Return actual radio frequency
double get_freq(struct demod * const demod){
  if(demod == NULL)
    return NAN;

  return demod->freq;
}

// Set a Doppler offset and sweep rate
int set_doppler(struct demod * const demod,double freq,double rate){
  assert(demod != NULL);
  set_osc(&demod->doppler, -freq/demod->input.samprate, -rate/(demod->input.samprate * demod->input.samprate));
  return 0;
}
double get_doppler(struct demod * const demod){
  assert(demod != NULL);
  pthread_mutex_lock(&demod->doppler.mutex);
  double f = demod->doppler.freq * demod->input.samprate;
  pthread_mutex_unlock(&demod->doppler.mutex);  
  return f;
}
double get_doppler_rate(struct demod * const demod){
  assert(demod != NULL);
  pthread_mutex_lock(&demod->doppler.mutex);
  double f = demod->doppler.rate * demod->input.samprate * demod->input.samprate;
  pthread_mutex_unlock(&demod->doppler.mutex);  
  return f;
}

// Set receiver frequency with optional first IF selection
// new_lo2 == NAN is a "don't care"; we'll try to pick a new LO2 that avoids retuning LO1.
// If that isn't possible we'll pick a default, (usually +/- 48 kHz, samprate/4)
// that moves the tuner the least
double set_freq(struct demod * const demod,double const f,double new_lo2){
  assert(demod != NULL);
  if(demod == NULL)
    return NAN;

  demod->freq = f;

  // No alias checking on explicitly provided lo2
  if(isnan(new_lo2) || !LO2_in_range(demod,new_lo2,0)){
    // Determine new LO2
    new_lo2 = -(f - get_first_LO(demod));
    // If the required new LO2 is out of range, retune LO1
    if(!LO2_in_range(demod,new_lo2,1)){
      // Pick new LO2 to minimize change in LO1 in case another receiver is using it
      new_lo2 = demod->status.samprate/4.;
      // Experimentally disable this to keep IF from jumping around when using mspectrum
#if 0
      double LO1 = get_first_LO(demod);

      if(fabs(f + new_lo2 - LO1) > fabs(f - new_lo2 - LO1))
	new_lo2 = -new_lo2;
#endif
    }
  }
  double new_lo1 = f + new_lo2;
  // returns actual frequency, which may be different from requested because
  // of calibration offset and quantization error in the fractional-N synthesizer
  double actual_lo1 = set_first_LO(demod,new_lo1);
  new_lo2 += (actual_lo1 - new_lo1); // fold the difference into LO2
    
  // If front end doesn't retune don't retune LO2 either (e.g., when receiving from a recording)
  if(LO2_in_range(demod,new_lo2,0))
    set_second_LO(demod,new_lo2);

  return f;
}

// Set first (front end tuner) oscillator
// Note: single precision floating point is not accurate enough at VHF and above
// demod->first_LO is NOT updated here!
// It is set by incoming status frames so this will take time
double set_first_LO(struct demod * const demod,double first_LO){
  assert(demod != NULL);
  if(demod == NULL)
    return NAN;

  double current_lo1 = get_first_LO(demod);

  // Just return actual frequency without changing anything
  if(first_LO == current_lo1 || first_LO <= 0 || demod->tuner_lock || demod->input.source_address.ss_family != AF_INET)
    return first_LO;

  demod->requested_status.frequency = first_LO;

  // These need a way to set
  // The Funcube dongle has a very wide dynamic range, so it is rarely necessary
  // to change its analog gain settings and the 'radio' program never does
  // Right now, gain is only changed inside the 'funcube' program in response to actual A/D overload, which is rare
  demod->requested_status.samprate = ADC_samprate; // Preferred samprate; ignored by funcube
  demod->requested_status.lna_gain = 0xff;    // 0xff means "don't change"
  demod->requested_status.mixer_gain = 0xff;
  demod->requested_status.if_gain = 0xff;
  
  // If we know the sender, send it a tuning request
  struct sockaddr_in sdraddr;
  memcpy(&sdraddr,&demod->input.source_address,sizeof(sdraddr));
  sdraddr.sin_port = htons(ntohs(sdraddr.sin_port)+1);
  if(sendto(demod->input.ctl_fd,&demod->requested_status,sizeof(demod->requested_status),0,(struct sockaddr *)&sdraddr,sizeof(sdraddr)) == -1)
    perror("sendto control socket");

  // Return the tuner's current frequency, which will change
  return demod->status.frequency;
}

// If avoid_alias is true, return 1 if specified carrier frequency is in range of LO2 given
// sampling rate, filter setting and alias region
//
// If avoid_alias is false, simply test that specified frequency is between +/- samplerate/2
int LO2_in_range(struct demod * const demod,double const f,int const avoid_alias){
  assert(demod != NULL);
  if(demod == NULL)
    return -1;

  if(avoid_alias)
    return f >= demod->min_IF + max(0.0f,demod->filter.high)
	    && f <= demod->max_IF + min(0.0f,demod->filter.low);
  else {
    return fabs(f) <=  0.5 * demod->input.samprate; // within Nyquist limit?
  }
}

// The next two frequency setting functions depend on the sample rate

// Set second local oscillator (the one in software)
// the caller must avoid aliasing, e.g., with LO2_in_range()
double set_second_LO(struct demod * const demod,double const second_LO){
  assert(demod != NULL);
  if(demod == NULL)
    return NAN;

  // In case sample rate isn't set yet, just remember the frequency but don't divide by zero
  if(second_LO == 0)
    set_osc(&demod->second_LO, 0.0, 0.0);
  else
    set_osc(&demod->second_LO,second_LO/demod->input.samprate, 0.0);
  return second_LO;
}

// Set audio frequency shift after downconversion and detection (linear modes only: SSB, IQ, DSB)
double set_shift(struct demod * const demod,double const shift){
  assert(demod != NULL);
  if(shift == 0)
    set_osc(&demod->shift, 0.0, 0.0);
  else 
    set_osc(&demod->shift,shift * demod->filter.decimate / (double)demod->input.samprate, 0.0);
  return shift;
}

// Set major operating mode
// This kills the current demodulator thread, sets up the predetection filter
// and other demodulator parameters, and starts the appropriate demodulator thread
int set_mode(struct demod * const demod,const char * const mode,int const defaults){
  assert(demod != NULL);
  if(demod == NULL)
    return -1;

  struct modetab *mp;
  for(mp = &Modes[0]; mp < &Modes[Nmodes]; mp++){
    if(strcasecmp(mode,mp->name) == 0)
      break;
  }
  if(mp == &Modes[Nmodes])
    return -1; // Unregistered mode

  // Kill current demod thread, if any, to cause clean exit
  demod->terminate = 1;
  pthread_join(demod->demod_thread,NULL); // Wait for it to finish
  demod->terminate = 0;

  // if the mode argument points to demod->mode, avoid the copy; can cause an abort
  if(demod->mode != mode)
    strlcpy(demod->mode,mode,sizeof(demod->mode));

  demod->demod_type = mp->demod_type;

  if(defaults || isnan(demod->filter.low) || isnan(demod->filter.high)){
    if(mp->low > mp->high){
      demod->filter.low = mp->high;
      demod->filter.high = mp->low;
    } else {
      demod->filter.low = mp->low;
      demod->filter.high = mp->high;
    }
  }

  demod->flat = mp->flat;
  demod->filter.isb = mp->isb;
  demod->output.channels = mp->channels;
  demod->pll = mp->pll;
  demod->square = mp->square;
  demod->agc.attack_rate = mp->attack_rate;
  demod->agc.recovery_rate = mp->recovery_rate;
  demod->agc.hangtime = mp->hangtime;
  
  if(defaults || isnan(demod->shift.freq)){
    if(demod->shift.freq != mp->shift){
      // Adjust tuning for change in frequency shift
      set_freq(demod,get_freq(demod) + mp->shift - demod->shift.freq,NAN);
    }
    set_shift(demod,mp->shift);
  }

  // Might now be out of range because of change in filter passband
  set_freq(demod,get_freq(demod),NAN);

  pthread_create(&demod->demod_thread,NULL,Demodtab[mp->demod_type].demod,demod);
  return 0;
}      


// Called from RTP receiver to process incoming metadata from SDR front end
void update_status(struct demod *demod,struct status *new_status){
  assert(demod != NULL);
  // Protect status with a mutex and signal a condition when it changes
  // since demod threads will be waiting for this
  int sig = 0;
  demod->status.timestamp = new_status->timestamp; // This should always change
  if(new_status->samprate != demod->status.samprate){
    // A/D sample rate is now known or has changed
    // This needs to be set before the demod thread starts!
    // Signalled every time the status is updated
    // status.samprate contains *nominal* A/D sample rate
    // Use nominal rates here so result is clean integer
    pthread_mutex_lock(&demod->status_mutex);
    demod->status.samprate = new_status->samprate;
    if(demod->status.samprate >= demod->output.samprate){
      // Sample rate is higher than audio rate; decimate
      demod->filter.interpolate = 1;
      demod->filter.decimate = demod->status.samprate / demod->output.samprate;
      demod->input.samprate = demod->status.samprate;
      demod->max_IF = IF_EXCLUDE * demod->status.samprate/2;
      demod->min_IF = -demod->max_IF;
    } else {
      // Sample rate is lower than audio rate
      // Interpolate up to audio rate, pretend sample rate is audio rate
      demod->filter.decimate = 1; 
      demod->filter.interpolate = demod->output.samprate / demod->status.samprate;	  
      demod->input.samprate = demod->output.samprate;
      demod->max_IF = IF_EXCLUDE * demod->output.samprate/2;
      demod->min_IF = -demod->max_IF;
    }
    // re-call these two to recalculate their phasor steps
    pthread_mutex_unlock(&demod->status_mutex);
    set_second_LO(demod,get_second_LO(demod));
	set_shift(demod,demod->shift.freq);
	sig++;
  }
  // Gain settings changed? Store and signal but take no other action for now
  if(new_status->lna_gain != demod->status.lna_gain){
    demod->status.lna_gain = new_status->lna_gain;
    sig++;
  }
  if(new_status->mixer_gain != demod->status.mixer_gain){
    demod->status.mixer_gain = new_status->mixer_gain;
    sig++;
  }
  if(new_status->if_gain != demod->status.if_gain){
    demod->status.if_gain = new_status->if_gain;
    sig++;
  }
  if(new_status->frequency != demod->status.frequency){
    pthread_mutex_lock(&demod->status_mutex);
	// Tuner is now set or has been changed
	// Adjust 2nd LO to compensate
	// NB! This may take the 2nd LO out of its range. This is deliberate so we don't get
	// into fights over the SDR tuner. If the tuner comes back, we'll recover
    demod->status.frequency = new_status->frequency;
    pthread_mutex_unlock(&demod->status_mutex);
    double new_LO2 = -(demod->freq - get_first_LO(demod));
    set_second_LO(demod,new_LO2);
    sig++;
  }
  if(sig){
    // Voltage gain to normalize signal power for analog gain settings
    demod->gain_factor = powf(10.,-0.05*(demod->status.lna_gain + demod->status.if_gain + demod->status.mixer_gain));
    // Something changed, store the new status and let everybody know
    pthread_mutex_lock(&demod->status_mutex);
    pthread_cond_broadcast(&demod->status_cond);
    pthread_mutex_unlock(&demod->status_mutex);
  }
}

// Compute noise spectral density - experimental, my algorithm
// The problem is telling signal from noise
// Heuristic: first average all bins outside the bandwidth
// Then recompute the average, tossing bins > 3 dB above the previous average
// Hopefully this will get rid of any signals from the noise estimate
float const compute_n0(struct demod const * const demod){
  assert(demod != NULL);
  if(demod == NULL || demod->filter.in == NULL)
    return NAN;
  
  struct filter_in const *f = demod->filter.in;
  int const N = f->ilen + f->impulse_length - 1;
  float power_spectrum[N];
  
  // Compute smoothed power spectrum
  // There will be some spectral leakage because the convolution FFT we're using is unwindowed
  // Includes both real and imaginary components, so this will have to be divided by 2 to get 0dBFS convention
  for(int n=0;n<N;n++)
    power_spectrum[n] = cnrmf(f->fdomain[n]);

  // compute average energy outside passband, then iterate computing a new average that
  // omits bins > 3dB above the previous average. This should pick up only the noise
  float avg_n = INFINITY;
  for(int iter=0;iter<2;iter++){
    int noisebins = 0;
    float new_avg_n = 0;
    for(int n=0;n<N;n++){
      float f;
      if(n <= N/2)
	f = (float)(n * demod->input.samprate) / N;
      else
	f = (float)((n-N) * demod->input.samprate) / N;
      
      if(f >= demod->filter.low && f <= demod->filter.high)
	continue; // Avoid passband
      
      float const s = power_spectrum[n];
      if(s < avg_n * 2){ // +3dB threshold
	new_avg_n += s;
	noisebins++;
      }
    }
    new_avg_n /= noisebins;
    avg_n = new_avg_n;
  }
  // return noise power per Hz, normalized to 0dBFS
  return avg_n / (2.0*N*demod->input.samprate);
}
