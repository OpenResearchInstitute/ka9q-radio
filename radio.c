// $Id: radio.c,v 1.113 2018/12/13 09:47:57 karn Exp karn $
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
#include "status.h"


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
    pthread_mutex_lock(&demod->input.qmutex);
    while(demod->input.queue == NULL)
      pthread_cond_wait(&demod->input.qcond,&demod->input.qmutex);
    pkt = demod->input.queue;
    demod->input.queue = pkt->next;
    pthread_mutex_unlock(&demod->input.qmutex);

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
	  demod->filter.out->out_type = demod->filter.isb ? CROSS_CONJ : COMPLEX;
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
      complex float samp = CMPLXF(samp_i,samp_q) * demod->sdr.gain_factor;
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
	demod->filter.out->out_type = demod->filter.isb ? CROSS_CONJ : COMPLEX;
	execute_filter_input(demod->filter.in);
	// Compute IF power and noise spectral density
	demod->sig.if_power = block_energy / in_cnt;
	block_energy = in_cnt = 0;
	if(!isnan(demod->sig.n0))
	  demod->sig.n0 += .005 * (compute_n0(demod) - demod->sig.n0);
	else
	  demod->sig.n0 = compute_n0(demod); // Happens at startup
      } // Every FFT block
    } // for each sample in I/Q packet
    free(pkt); pkt = NULL;
  } // end of main loop
}

// Get true first LO frequency, with TCXO offset applied
double const get_first_LO(const struct demod * const demod){
  if(demod == NULL)
    return NAN;
	 
  return demod->sdr.status.frequency;
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

  return demod->tune.freq;
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

  assert(!isnan(f));
  assert(f != 0);

  demod->tune.freq = f;

  // No alias checking on explicitly provided lo2
  if(isnan(new_lo2) || !LO2_in_range(demod,new_lo2,0)){
    // Determine new LO2
    new_lo2 = -(f - get_first_LO(demod));
    // If the required new LO2 is out of range, retune LO1
    if(!LO2_in_range(demod,new_lo2,1)){
      // Pick new LO2 to minimize change in LO1 in case another receiver is using it
      new_lo2 = demod->sdr.status.samprate/4.;
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
  if(first_LO == current_lo1 || first_LO <= 0 || demod->tune.lock)
    return first_LO;

  unsigned char packet[8192],*bp;
  memset(packet,0,sizeof(packet));
  bp = packet;
  *bp++ = 1; // Command
  demod->input.command_tag = random();
  encode_int32(&bp,COMMAND_TAG,demod->input.command_tag);
  encode_double(&bp,RADIO_FREQUENCY,first_LO);
  encode_eol(&bp);
  int len = bp - packet;
  int i = send(demod->input.ctl_fd,packet,len,0);
  assert(i>0);
  return first_LO;
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
    return f >= demod->sdr.min_IF + max(0.0f,demod->filter.high)
	    && f <= demod->sdr.max_IF + min(0.0f,demod->filter.low);
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
  demod->tune.shift = shift;
  if(shift == 0)
    set_osc(&demod->shift, 0.0, 0.0);
  else 
    set_osc(&demod->shift,shift * demod->filter.decimate / (double)demod->input.samprate, 0.0);
  return shift;
}

double get_shift(struct demod * const demod){
  assert(demod != NULL);
  return demod->tune.shift;
}


// Load mode table entry presets without actually engaging
// Return nonzero if demod has changed and must be engaged
int preset_mode(struct demod * const demod,const char * const mode){
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

  if(mp->low > mp->high){
    demod->filter.low = mp->high;
    demod->filter.high = mp->low;
  } else {
    demod->filter.low = mp->low;
    demod->filter.high = mp->high;
  }
  demod->tune.shift = mp->shift;
  demod->opt.flat = mp->flat;
  demod->filter.isb = mp->isb;
  demod->output.channels = mp->channels;
  demod->opt.env = mp->envelope;
  demod->opt.pll = mp->pll;
  demod->opt.square = mp->square;
  demod->agc.attack_rate = mp->attack_rate;
  demod->agc.recovery_rate = mp->recovery_rate;
  demod->agc.hangtime = mp->hangtime;
  if(demod->demod_type != mp->demod_type){
    pthread_mutex_lock(&demod->demod_mutex);
    demod->demod_type = mp->demod_type;
    pthread_cond_broadcast(&demod->demod_cond);
    pthread_mutex_unlock(&demod->demod_mutex);
  }
  set_shift(demod,demod->tune.shift);
  set_filter(demod->filter.out,
	     demod->filter.low/demod->output.samprate,
	     demod->filter.high/demod->output.samprate,
	     demod->filter.kaiser_beta);
  return 0;
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
  // Includes both real and imaginary components
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
  return avg_n / (N*demod->input.samprate);
}
