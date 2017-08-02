#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <math.h>
#include <complex.h>
#undef I
#include <fftw3.h>

#include "dsp.h"
#include "filter.h"
#include "radio.h"
#include "audio.h"

void *demod_cam(void *arg){
  pthread_setname("cam");
  assert(arg != NULL);
  struct demod * const demod = arg;

  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,COMPLEX);
  demod->filter = filter;
  set_filter(demod,demod->low,demod->high);

  complex float lastphase = 0;
  while(fillbuf(demod->corr_iq_read_fd,filter->input.c,filter->ilen*sizeof(*filter->input.c)) > 0){
    spindown(demod,filter->input.c,filter->ilen); // 2nd LO
    execute_filter(filter);

    // Grab carrier phase from DC bin of frequency domain
    complex float phase = filter->f_fdomain[0];
    phase = conj(phase) / cabs(phase);
    // Rotate signal onto I axis, measure DC (carrier) level
    float amplitude = 0;
    float noise = 0;
    int n;
    for(n=0; n < filter->olen; n++){
      // Sample with signal rotated onto I axis
      complex float rsamp = filter->output.c[n] *= phase;
      amplitude += creal(rsamp) * creal(rsamp);
      noise += cimag(rsamp) * cimag(rsamp);
    }
    // RMS signal+noise and noise amplitudes
    demod->amplitude = sqrtf(amplitude / filter->olen);
    demod->noise = sqrtf(noise / filter->olen);
    float const snn = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio
    demod->snr = (snn*snn) -1; // S/N as power ratio

    // Frequency error is the phase of this block minus the last, times blocks/sec
    // Phase was already flipped, hence the minus
    // Only move a fraction of the error at one time
    double const freqerror = -0.01 * carg(phase * conj(lastphase)) * M_1_2PI * demod->samprate/filter->ilen;
    lastphase = phase;
    set_second_LO(demod,-freqerror + demod->second_LO);

    // Remove carrier DC
    // AM AGC is carrier-driven
    //    demod->gain = Headroom / demod->amplitude;
    demod->gain = 0.5/demod->amplitude;

    float audio[filter->olen];
    for(n=0; n < filter->olen; n++)
      audio[n] = demod->gain * (creal(filter->output.c[n]) - demod->amplitude);

    send_mono_audio(demod->audio,audio,n);
  }
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
