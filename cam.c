#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <math.h>
#include <complex.h>
#undef I
#include <fftw3.h>
#include <string.h>

#include "dsp.h"
#include "filter.h"
#include "radio.h"
#include "audio.h"

void *demod_cam(void *arg){
  pthread_setname("cam");
  assert(arg != NULL);
  struct demod * const demod = arg;

  assert(demod->decimate == 4);
  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,COMPLEX);
  demod->filter = filter;
  set_filter(filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);

  if(demod->flags & CAL)
    demod->demod_offset = 0; // Force to zero, we'll adjust calibration
  complex float lastphase = 0;
  memset(filter->input_buffer.c,0,(filter->impulse_length - 1)*sizeof(complex float)); 
  while(!demod->terminate){
    fillbuf(demod,filter->input.c,filter->ilen);
    demod->second_LO_phasor = spindown(demod,filter->input.c); // 2nd LO
    demod->if_power = cpower(filter->input.c,filter->ilen);
    execute_filter(filter);
    demod->bb_power = cpower(filter->output.c,filter->olen);
    demod->n0 += .01 * (compute_n0(demod) - demod->n0);

    // Get DC (carrier) phasor by averaging
    complex float phase = 0;
    {
      int n;
      for(n=0;n<filter->olen;n++)
	phase += filter->output.c[n];
      float amp = cabsf(phase);
      if(amp == 0)
	continue; // Probably no signal; skip
      phase /= amp;
    }

    assert(!isnan(crealf(phase)) && !isnan(cimagf(phase)));
    
    phase = conj(phase);
    // Rotate onto I axis
    {
      int n;
      for(n=0;n<filter->olen;n++)
	filter->output.c[n] *= phase;
    }
    // measure sighal (I), noise (Q) and DC (carrier) levels
    float amplitude;
    {
      complex double const powers = cpowers(filter->output.c,filter->olen);
      amplitude = crealf(powers);
      float const noise = cimagf(powers);
      demod->snr = (amplitude / noise) - 1; // S/N as power ratio
    }

    amplitude = sqrtf(amplitude); // Convert to magnitude

    // Frequency error is the phase of this block minus the last, times blocks/sec
    // Phase was already flipped, hence the minus
    // Only move a fraction of the error at one time
    double const freqerror = -0.01 * carg(phase * conj(lastphase)) * M_1_2PI * demod->samprate/filter->ilen;
    assert(!isnan(freqerror));
    lastphase = phase;
    if(demod->flags & CAL){
      // Keep same dial frequency, adjust calibration with stiffer response
      if(demod->frequency != 0)
	set_cal(demod,demod->calibrate - freqerror/(10. * demod->frequency));
    } else {
      // Retune second LO (and RF frequency)
      //      set_offset(demod,demod->demod_offset + freqerror);  // REDO THIS!!!
    }

    // Remove carrier DC
    // AM AGC is carrier-driven
    demod->gain = demod->headroom / amplitude;

    float audio[filter->olen];
    int n;
    for(n=0; n < filter->olen; n++)
      audio[n] = creal(filter->output.c[n]) - amplitude;
    send_mono_audio(demod->audio,audio,filter->olen,demod->gain);
  }
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
