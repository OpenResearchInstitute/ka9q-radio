// $Id: am.c,v 1.40 2018/12/06 09:45:29 karn Exp karn $
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
  float const recovery_factor = dB2voltage(demod->agc.recovery_rate * samptime); // AGC ramp-up rate/sample
  //  float const attack_factor = dB2voltage(demod->agc.attack_rate * samptime);      // AGC ramp-down rate/sample
  int const hangmax = demod->agc.hangtime / samptime; // samples before AGC increase

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

    // New samples
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
	hangcount = hangmax;
      } else if(hangcount != 0){
	hangcount--;
      } else {
	demod->agc.gain *= recovery_factor;
      }
      samples[n] = (samp - DC_filter) * demod->agc.gain;
    }
    send_mono_output(demod,samples,filter->olen);
    // Scale to each sample so baseband power will display correctly
    demod->sig.bb_power = signal / filter->olen;
  }
}
