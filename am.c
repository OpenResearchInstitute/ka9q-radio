// $Id: am.c,v 1.42 2018/12/10 11:54:05 karn Exp karn $
// AM envelope demodulator thread for 'radio'
// Copyright Oct 9 2017, Phil Karn, KA9Q
#define _GNU_SOURCE 1
#include <complex.h>
#include <math.h>
#include <assert.h>
#include <pthread.h>

#include "misc.h"
#include "dsp.h"
#include "filter.h"
#include "radio.h"

void *demod_am(void *arg){
  pthread_setname("am");
  assert(arg != NULL);
  struct demod * const demod = arg;

  // Set derived (and other) constants
  float const samptime = demod->filter.decimate / (float)demod->input.samprate;  // Time between (decimated) samples

  // AGC
  // I originally just kept the carrier at constant amplitude
  // but this fails when selective fading takes out the carrier, resulting in loud, distorted audio
  int hangcount = 0;
#if 0
  float attack_rate_per_sample = dB2voltage(demod->agc.attack_rate * samptime);
  float prev_attack_rate = demod->agc.attack_rate;
#endif
  float recovery_rate_per_sample = dB2voltage(demod->agc.recovery_rate * samptime);
  float prev_recovery_rate = demod->agc.recovery_rate;

  // DC removal from envelope-detected AM and coherent AM
  float DC_filter = 0;
  float const DC_filter_coeff = .0001;

  demod->output.channels = 1; // Mono

  struct filter_out * const filter = demod->filter.out;

  while(1){
    // Are we active?
    pthread_mutex_lock(&demod->demod_mutex);
    while(demod->demod_type != AM_DEMOD)
      pthread_cond_wait(&demod->demod_cond,&demod->demod_mutex);
    pthread_mutex_unlock(&demod->demod_mutex);

    // Recompute AGC if it has changed
#if 0
    if(demod->agc.attack_rate != prev_attack_rate){
      attack_rate_per_sample = dB2voltage(demod->agc.attack_rate * samptime);
      prev_attack_rate = demod->agc.attack_rate;
    }
#endif
    if(demod->agc.recovery_rate != prev_recovery_rate){
      recovery_rate_per_sample = dB2voltage(demod->agc.recovery_rate * samptime);
      prev_recovery_rate = demod->agc.recovery_rate;
    }

    // Wait for new samples
    execute_filter_output(filter);    

    // AM envelope detector
    float signal = 0;
    float samples[filter->olen];
    for(int n=0; n<filter->olen; n++){
      float const sampsq = cnrmf(filter->output.c[n]);
      signal += sampsq;
      float samp = sqrtf(sampsq);

      // Remove carrier DC from audio
      // DC_filter will always be positive since sqrtf() is positive
      DC_filter += DC_filter_coeff * (samp - DC_filter);
      
      if(isnan(demod->agc.gain)){
	demod->agc.gain = demod->agc.headroom / DC_filter;
      } else if(demod->agc.gain * DC_filter > demod->agc.headroom){
	demod->agc.gain = demod->agc.headroom / DC_filter;
	hangcount = demod->agc.hangtime * demod->output.samprate;
      } else if(hangcount != 0){
	hangcount--;
      } else {
	demod->agc.gain *= recovery_rate_per_sample;
      }
      samples[n] = (samp - DC_filter) * demod->agc.gain;
    }
    send_mono_output(demod,samples,filter->olen);
    // Scale to each sample so baseband power will display correctly
    demod->sig.bb_power = signal / filter->olen;
  }
}
