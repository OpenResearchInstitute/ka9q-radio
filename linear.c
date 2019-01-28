// $Id: linear.c,v 1.45 2019/01/14 10:33:42 karn Exp karn $

// General purpose linear demodulator
// Handles USB/IQ/CW/etc, basically all modes but FM and envelope-detected AM
// Copyright Sept 20 2017 Phil Karn, KA9Q

#define _GNU_SOURCE 1
#include <assert.h>
#include <complex.h>
#include <math.h>
#include <fftw3.h>
#include <pthread.h>
#include <string.h>

#include "misc.h"
#include "dsp.h"
#include "filter.h"
#include "radio.h"


void *demod_linear(void *arg){
  pthread_setname("linear");
  assert(arg != NULL);
  struct demod * const demod = arg;

  demod->opt.loop_bw = 5; // eventually to be set from mode table

  // AGC
  int hangcount = 0;

  // Coherent mode parameters
  float const snrthresh = 2;     // Loop lock threshold at +3 dB SNR
  float const damping = M_SQRT1_2; // PLL loop damping factor; 1/sqrt(2) is "critical" damping
  float const lock_time = 1;       // hysteresis parameter: 2*locktime seconds of good signal -> lock, 2*locktime sec of bad signal -> unlock

  const int lock_limit = lock_time * demod->output.samprate;

  int lock_count = 0;

  struct pll pll;
  init_pll(&pll,demod->opt.loop_bw,damping,0.0,(float)demod->output.samprate);


  demod->sig.snr = 0;

  // Detection filter
  struct filter_out * const filter = demod->filter.out;

  while(1){
    // Are we active?
    pthread_mutex_lock(&demod->demod_mutex);
    while(demod->demod_type != LINEAR_DEMOD)
      pthread_cond_wait(&demod->demod_cond,&demod->demod_mutex);
    pthread_mutex_unlock(&demod->demod_mutex);

    // Wait for new samples
    execute_filter_output(filter,0);

    
    if(demod->opt.pll){
      // Loop lock detector with hysteresis
      // If the loop is locked, the SNR must fall below the threshold for a while
      // before we declare it unlocked, and vice versa
      if(demod->sig.snr < snrthresh){
	lock_count -= filter->olen;
      } else {
	lock_count += filter->olen;
      }
      if(lock_count >= lock_limit){
	lock_count = lock_limit;
	demod->sig.pll_lock = 1;
      }
      if(lock_count <= -lock_limit){
	lock_count = -lock_limit;
	demod->sig.pll_lock = 0;
      }
      demod->sig.lock_timer = lock_count;
      
      float signal = 0;
      float noise = 0;
      for(int n=0;n<filter->olen;n++){
	complex float s = filter->output.c[n];
	s *= conjf(pll.vco.phasor);
	float phase;
	if(demod->opt.square){
	  phase = cargf(s*s);
	} else {
	  phase = cargf(s);
	}
	run_pll(&pll,phase);
	filter->output.c[n] = s;
	
	float rp = crealf(s) * crealf(s);
	float ip = cimagf(s) * cimagf(s);
	signal += rp;
	noise += ip;
      }
      demod->sig.cphase = carg(pll.vco.phasor);
      if(demod->opt.square)
	demod->sig.cphase /= 2; // Squaring doubles the phase

      demod->sig.foffset = pll.vco.freq * demod->output.samprate;
      if(noise != 0){
	demod->sig.snr = (signal / noise) - 1; // S/N as power ratio; meaningful only in coherent modes
	if(demod->sig.snr < 0)
	  demod->sig.snr = 0; // Clamp to 0 so it'll show as -Inf dB
      } else
	demod->sig.snr = NAN;
    }
    // Demodulation
    float samples[filter->olen]; // for mono output
    float energy = 0;
    float output_level = 0;
    for(int n=0; n<filter->olen; n++){
      complex float s = filter->output.c[n] * step_osc(&demod->shift);
      float norm = cnrmf(s);
      energy += norm;
      float amplitude = sqrtf(norm);
      
      // AGC
      // Lots of people seem to have strong opinions how AGCs should work
      // so there's probably a lot of work to do here
      // The attack_factor feature doesn't seem to work well; if it's at all
      // slow you get an annoying "pumping" effect.
      // But if it's too fast, brief spikes can deafen you for some time
      // What to do?
      if(demod->opt.agc){
	if(isnan(demod->agc.gain) || amplitude * demod->agc.gain > demod->agc.headroom){
	  demod->agc.gain = demod->agc.headroom / amplitude; // Startup
	  hangcount = demod->agc.hangtime;
	} else if(hangcount > 0){
	  hangcount--;
	} else {
	  demod->agc.gain *= demod->agc.recovery_rate;
	}
      }
      if(demod->opt.env){
	// AM envelope detection -- should re-add DC removal
	samples[n] = amplitude * demod->agc.gain;
	output_level += samples[n] * samples[n];
      } else if(demod->output.channels == 1) {
	samples[n] = crealf(s) * demod->agc.gain;
	output_level += samples[n] * samples[n];
      } else {
	filter->output.c[n] = s * demod->agc.gain;
	output_level += cnrmf(filter->output.c[n]);
      }
    }
    demod->output.level = output_level / (filter->olen * demod->output.channels);
    if(demod->opt.env || demod->output.channels == 1)
      send_mono_output(demod,samples,filter->olen);
    else      // I on left, Q on right
      send_stereo_output(demod,(float *)filter->output.c,filter->olen);

    // Total baseband power (I+Q), scaled to each sample
    demod->sig.bb_power = energy / filter->olen;
  }
}
