// $Id: radio.c,v 1.122 2018/12/28 10:16:17 karn Exp karn $
// Core of 'radio' program - control LOs, set frequency/mode, etc
// Copyright 2018, Phil Karn, KA9Q
#define _GNU_SOURCE 1
#include <assert.h>
#include <stdlib.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
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


// thread for first half of demodulator
// Preprocessing of samples performed for all demodulators
// Pass to input of pre-demodulation filter
// Update power measurement, estimate noise level

float const SCALE16 = 1./SHRT_MAX; // Scale signed 16-bit int to float in range -1, +1
float const SCALE8 = 1./127;       // Scale signed 8-bit int to float in range -1, +1

void *proc_samples(void *arg){
  assert(arg);
  pthread_setname("procsamp");

  struct demod *demod = (struct demod *)arg;
  float block_energy = 0;
  int in_cnt = 0;

  while(1){
    // Packet consists of Ethernet, IP and UDP header (already stripped)
    // then standard Real Time Protocol (RTP), a status header and the PCM
    // I/Q data. RTP is an IETF standard, so it uses big endian numbers
    // The status header and I/Q data are *not* standard, so we save time
    // by using machine byte order (almost certainly little endian).
    // Note this is a portability problem if this system and the one generating
    // the data have opposite byte orders. But who's big endian anymore?
    // Receive I/Q data from front end
    // Incoming RTP packets

    struct packet pkt;

    socklen_t socksize = sizeof(demod->input.data_source_address);
    int size = recvfrom(demod->input.data_fd,pkt.content,sizeof(pkt.content),0,(struct sockaddr *)&demod->input.data_source_address,&socksize);
    if(size <= 0){    // ??
      perror("recvfrom");
      usleep(50000);
      continue;
    }
    if(size < RTP_MIN_SIZE)
      continue; // Too small for RTP, ignore

    unsigned char *dp = pkt.content;
    dp = ntoh_rtp(&pkt.rtp,dp);
    size -= (dp - pkt.content);
    
    if(pkt.rtp.pad){
      // Remove padding
      size -= dp[size-1];
      pkt.rtp.pad = 0;
    }
    int sampcount;

    switch(pkt.rtp.type){
    case IQ_PT: // Little-endian 16 bit ints with old metadata header
      dp += 24;
      size -= 24;
      sampcount = size / (2 * sizeof(signed short));
      break;
    case IQ_PT8: // 8-bit ints no metadata
      sampcount = size / (2 * sizeof(signed char));
      break;
    case PCM_STEREO_PT: // Big-endian 16 bits, no metadata header
      sampcount = size / (2 * sizeof(signed short));
      break;
    case IQ_PT12:       // Big endian packed 12 bits, no metadata
      sampcount = size / 3;
      break;
    default:
      continue; // Unsupported type; ignore
    }
    pkt.data = dp;
    pkt.len = size;

    if(pkt.rtp.ssrc != demod->input.rtp.ssrc){
      // SSRC changed; reset sample count.
      // rtp_process will reset packet count
      demod->input.samples = 0;
    }
    int time_step = rtp_process(&demod->input.rtp,&pkt.rtp,sampcount);
    if(time_step < 0 || time_step > 192000){
      // Old samples, or too big a jump; drop. Shouldn't happen if sequence number isn't old
      continue;
    } else if(time_step > 0){
      // Samples were lost. Inject enough zeroes to keep the sample count and LO phase correct
      // Arbitrary 1 sec limit just to keep things from blowing up
      // Good enough for the occasional lost packet or two
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
    // Convert and scale samples to internal float-32 format
    demod->input.samples += sampcount;
    complex float sampbuf[sampcount];

    switch(pkt.rtp.type){
    default: // shuts up lint
    case IQ_PT12:      // two 12-bit signed integers packed big-endian into 3 bytes
      {
	float gain = SCALE16 * demod->sdr.gain_factor;
	for(int i=0; i<sampcount; i++){
	  __real__ sampbuf[i] = gain * (short)(((dp[0] << 8) | dp[1]) & 0xfff0);
	  __imag__ sampbuf[i] = gain * (short)(((dp[1] << 8) | dp[2]) << 4);
	  dp += 3;
	}
      }
      break;
    case PCM_STEREO_PT:      // Two 16-bit signed integers, BIG ENDIAN (network order)
      {
	signed short *sp = (signed short *)dp;
	float gain = SCALE16 * demod->sdr.gain_factor;
	for(int i=0; i<sampcount; i++){
	  __real__ sampbuf[i] = gain * ntohs(*sp++);
	  __imag__ sampbuf[i] = gain * ntohs(*sp++);
	}
      }
      break;
    case IQ_PT:      // Two 16-bit signed integers LITTLE ENDIAN
      {
	signed short *sp = (signed short *)dp;
	float gain = SCALE16 * demod->sdr.gain_factor;
	for(int i=0; i<sampcount; i++){
	  __real__ sampbuf[i] = gain * *sp++;
	  __imag__ sampbuf[i] = gain * *sp++;
	}
      }
      break;
    case IQ_PT8:      // Two signed 8-bit integers
      {
	float gain = SCALE8 * demod->sdr.gain_factor;
	for(int i=0; i<sampcount; i++){
	  __real__ sampbuf[i] = gain * (char)*dp++;
	  __imag__ sampbuf[i] = gain * (char)*dp++;
	}
      }
      break;
    }
    // Apply Doppler if active
    if(demod->doppler.freq != 0){
      for(int i=0; i < sampcount; i++){
	sampbuf[i] *= step_osc(&demod->doppler);
      }
    }

    for(int i=0; i < sampcount; i++){
      complex float samp = sampbuf[i] * step_osc(&demod->second_LO);      // Mix down
      block_energy += cnrmf(samp);

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
      if(!demod->sdr.direct_conversion)
	new_lo2 = 0; // No need to avoid DC
      else {
	// Pick new LO2 to minimize change in LO1 in case another receiver is using it
	new_lo2 = demod->input.samprate/4.;  // Halfway between DC and Nyquist rate
	double LO1 = get_first_LO(demod);
	if(fabs(f + new_lo2 - LO1) > fabs(f - new_lo2 - LO1))
	  new_lo2 = -new_lo2;
      }
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
  send(demod->input.ctl_fd,packet,len,0);
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
    set_osc(&demod->shift,shift / (double)demod->output.samprate, 0.0);
  return shift;
}

double get_shift(struct demod * const demod){
  assert(demod != NULL);
  return demod->tune.shift;
}

// Load mode table entry presets
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
  demod->opt.env = mp->env;
  demod->opt.pll = mp->pll;
  demod->opt.square = mp->square;
  // dB/sec -> voltage ratio/sample
  demod->agc.attack_rate = dB2voltage(mp->attack_rate / demod->output.samprate);
  demod->agc.recovery_rate = dB2voltage(mp->recovery_rate / demod->output.samprate);
  // time in seconds -> samples
  demod->agc.hangtime = mp->hangtime * demod->output.samprate;
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

  int low_n = (N * demod->filter.low) / demod->input.samprate;
  int high_n = (N * demod->filter.high) / demod->input.samprate;
  int bincount = N - (high_n - low_n);
  if(bincount < 0 || bincount > N){
    return 0; // Can this happen?
  }
  // Compute smoothed power spectrum
  // There will be some spectral leakage because the convolution FFT we're using is unwindowed
  // Includes both real and imaginary components
  float power_spectrum[bincount];
  int in = high_n + 1;
  for(int n=0; n < bincount; n++){ // modulo operation without division
    if(in >= N)
      in -= N;
    else if(in < 0)
      in += N;
    power_spectrum[n] = cnrmf(f->fdomain[in++]);
  }
  // compute average energy outside passband, then iterate computing a new average that
  // omits bins > 3dB above the previous average. This should pick up only the noise
  float avg_n = INFINITY;
  for(int iter=0;iter<2;iter++){
    int noisebins = 0;
    float new_avg_n = 0;
    for(int n=0; n < bincount; n++){
      float const s = power_spectrum[n];
      if(s < avg_n * 2){ // +3dB threshold
	new_avg_n += s;
	noisebins++;
      }
    }
    avg_n = new_avg_n / noisebins;
  }
  // return noise power per Hz, normalized to 0dBFS
  return avg_n / ((float)N*demod->input.samprate); // integer product can overflow 32 bits for large N and samprate
}
