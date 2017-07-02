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
  assert(arg != NULL);

  struct demod * const demod = arg;

  pthread_setname_np(pthread_self(),"cam");
  demod->foffset = NAN; // not used
  demod->pdeviation = NAN;

  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX);
  demod->filter = filter;
  set_filter(demod,demod->low,demod->high);

  complex float lastphase = 0;
  while(!demod->terminate){
    fillbuf(demod->input,(char *)filter->input,
	    filter->blocksize_in*sizeof(complex float));
    spindown(demod,filter->input,filter->blocksize_in); // 2nd LO
    execute_filter(filter);

    // Automatic gain control
    complex float phase;
    int n;
    float amplitude,noise;

    double freqerror;
    
    phase = 0;
    for(n=0; n < filter->blocksize_out; n++)
      phase += filter->output.c[n];

    phase = conj(phase) / cabs(phase);

    // Rotate signal onto I axis, measure DC (carrier) level
    amplitude = 0;
    noise = 0;
    for(n=0; n < filter->blocksize_out; n++){
      // Sample with signal rotated onto I axis
      complex float rsamp = filter->output.c[n] *= phase;
      amplitude += creal(rsamp) * creal(rsamp);
      noise += cimag(rsamp) * cimag(rsamp);
    }
    // RMS signal+noise and noise amplitudes
    demod->amplitude = sqrtf(amplitude / filter->blocksize_out);
    demod->noise = sqrtf(noise / filter->blocksize_out);
    float snn = demod->amplitude / demod->noise; // (S+N)/N amplitude ratio
    demod->snr = (snn*snn) -1; // S/N as power ratio

    // Frequency error is the phase of this block minus the last, times blocks/sec
    // Phase was already flipped, hence the minus
    // Only move a fraction of the error at one time
    freqerror = -0.01 * carg(phase * conj(lastphase)) * M_1_2PI * demod->samprate/filter->blocksize_in;
    lastphase = phase;
    set_second_LO(demod,-freqerror + demod->second_LO);

    // Remove carrier DC
    // AM AGC is carrier-driven
    demod->gain = Headroom / demod->amplitude;

    float audio[filter->blocksize_out];
    for(n=0; n < filter->blocksize_out; n++)
      audio[n] = demod->gain * (creal(filter->output.c[n]) - demod->amplitude);

    send_mono_audio(audio,n);
  }
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}
