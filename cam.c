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

  {
    int i;
    for(i=0;i<filter->ilen;i++)
      assert(!isnan(crealf(filter->response[i])) && !isnan(cimagf(filter->response[i])));
  }

  complex float lastphase = 0;
  memset(filter->input_buffer.c,0,(filter->impulse_length - 1)*sizeof(complex float)); 
  while(!demod->terminate){
    fillbuf(demod,filter->input.c,filter->ilen);
    {
      int i;
      for(i=0;i<filter->ilen;i++)
	assert(!isnan(crealf(filter->input.c[i])) && !isnan(cimagf(filter->input.c[i])));
    }

    spindown(demod,filter->input.c,filter->ilen); // 2nd LO
    {
      int i;
      for(i=0;i<filter->ilen;i++)
	assert(!isnan(crealf(filter->input.c[i])) && !isnan(cimagf(filter->input.c[i])));
    }
    execute_filter(filter);
    {
      int i;
      for(i=0;i<filter->olen;i++)
	assert(!isnan(crealf(filter->output.c[i])) && !isnan(cimagf(filter->output.c[i])));
    }

    // Grab carrier phase from DC bin of frequency domain
    complex float phase = filter->f_fdomain[0];
    assert(!isnan(crealf(phase)) && !isnan(cimagf(phase)));
    
    float amp = cabsf(phase);
    if(amp == 0)
      continue; // All zeroes? skip
    phase = conj(phase) / amp;
    // Rotate signal onto I axis, measure sighal (I), noise (Q) and DC (carrier) levels
    float amplitude = 0;
    float noise = 0;
    int n;
    for(n=0; n < filter->olen; n++){
      // Sample with signal rotated onto I axis
      complex float const rsamp = filter->output.c[n] *= phase;
      amplitude += crealf(rsamp) * crealf(rsamp);
      noise += cimagf(rsamp) * cimagf(rsamp);
    }
    // RMS signal+noise and noise amplitudes
    amplitude /= filter->olen;
    noise /= filter->olen;
    demod->amplitude = sqrtf(amplitude); // RMS amplitude of I channel
    demod->noise = sqrtf(noise);         // RMS amplitude of Q channel
    demod->snr = (amplitude / noise) - 1; // S/N as power ratio

    // Frequency error is the phase of this block minus the last, times blocks/sec
    // Phase was already flipped, hence the minus
    // Only move a fraction of the error at one time
    double const freqerror = -0.01 * carg(phase * conj(lastphase)) * M_1_2PI * demod->samprate/filter->ilen;
    assert(!isnan(freqerror));
    lastphase = phase;
    set_second_LO(demod,-freqerror + demod->second_LO);

    // Remove carrier DC
    // AM AGC is carrier-driven
    demod->gain = demod->headroom / demod->amplitude;

    float audio[filter->olen];
    for(n=0; n < filter->olen; n++)
      audio[n] = demod->gain * (creal(filter->output.c[n]) - demod->amplitude);
    send_mono_audio(demod->audio,audio,n);
  }
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
