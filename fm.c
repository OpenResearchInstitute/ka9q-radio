// $Id: fm.c,v 1.23 2017/07/08 20:35:27 karn Exp karn $
// FM demodulation and squelch
#define _GNU_SOURCE 1
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>
#undef I

#include "dsp.h"
#include "filter.h"
#include "radio.h"
#include "audio.h"

// Estimate FM SNR
float fm_snr(struct demod *demod,complex float *buffer,int L){
  // Find average magnitude and magnitude^2
  // Approximate because magnitude has a chi-squared distribution with 2 degrees of freedom
  float avg_squares = 0;
  float avg = 0;
  int n;
  for(n=0;n<L;n++){
    float const magsq = cnrmf(buffer[n]); // I^2 + Q^2
    avg_squares += magsq;
    avg += sqrtf(magsq);            // magnitude
  }
  avg_squares /= L; // Average square magnitude
  avg /= L;         // Average magnitude
  demod->amplitude = avg;
  float const fm_variance = avg_squares - avg*avg;
  demod->noise = sqrtf(fm_variance);
  
  // Find SNR
  return avg*avg/(2*fm_variance) - 1;
}


// in-place FM demodulator
// Note: output can range from -PI to +PI; do gain scaling appropriately
int do_fm(float *output,complex float const *buffer,int L,float *state){
  int n;

  output[0] = cargf(buffer[0] * conjf(*state));
  for(n=1; n<L; n++)
    output[n] = cargf(buffer[n] * conj(buffer[n-1]));

  *state = buffer[L-1];
  return 0;
}



void *demod_fm(void *arg){
  assert(arg != NULL);
  struct demod * const demod = arg;  
  complex float state = 0;
  pthread_setname_np(pthread_self(),"fm");
  float const dsamprate = demod->samprate / demod->decimate; // Decimated (output) sample rate

  demod->pdeviation = 0;
  demod->foffset = 0;

  // Create predetection filter, leaving response momentarily empty
  complex float * const response = fftwf_alloc_complex(demod->L+demod->M-1);

  struct filter * const filter = create_filter(demod->L,demod->M,response,demod->decimate,COMPLEX,COMPLEX);
  demod->filter = filter;
  set_filter(demod,demod->low,demod->high); // Set response

  // Set up audio baseband filter
  // Response is high pass with 300 Hz corner, then -6 dB/octave post-emphasis since demod is FM
  // and modulation is probably PM (indirect FM)
  const int AL = demod->L / demod->decimate;
  const int AM = (demod->M - 1) / demod->decimate + 1;
  const int AN = AL + AM - 1;

  // Positive frequencies only
  complex float * const aresponse = fftwf_alloc_complex(AN/2+1);
  memset(aresponse,0,(AN/2+1) * sizeof(*aresponse));
  float gain = 1./AN;
  int j;
  for(j=0;j<=AN/2;j++){
    float const f = (float)j * dsamprate / AN;
    if(f >= 300 && f <= 6000)
      aresponse[j] = gain * 300./f; // -6 dB/octave de-emphasis to handle PM (indirect FM) transmission
  }
  // Window scaling for REAL input, REAL output
  window_rfilter(AL,AM,aresponse,Kaiser_beta);

  struct filter * const afilter = create_filter(AL,AM,aresponse,1,REAL,REAL); // Real input, real output, same sample rate

  // PL decoder
  const int PL_decimate = 32; // 48 kHz in, 1500 Hz out
  const int PL_samprate = dsamprate / PL_decimate;
  const int PL_L = AL / PL_decimate;
  const int PL_M = (AM - 1) / PL_decimate + 1;
  const int PL_N = PL_L + PL_M - 1;

  complex float * const plresponse = fftwf_alloc_complex(PL_N);
  memset(plresponse,0,PL_N*sizeof(*plresponse));
  // Positive frequencies only, so we generate the analytic signal between 0 and 300 Hz
  // Gain is still 1./AN
  for(j=0;j<=PL_N/2;j++){
    float const f = (float)j * dsamprate / AN; // frequencies relative to INPUT sampling rate
    if(f > 0 && f < 300)
      plresponse[j] = gain;
  }
  window_filter(PL_L,PL_M,plresponse,1.0);
  // Create analytic signal of PL tone range in baseband
  struct filter * const plfilter = create_filter(AL,AM,plresponse,PL_decimate,REAL,COMPLEX);

  float lastaudio = 0;

  while(!demod->terminate){
    // Constant gain used by FM only; automatically adjusted by AGC in linear modes
    // We do this in the loop because BW can change
    demod->gain = (Headroom *  M_1_PI * dsamprate) / fabsf(demod->low - demod->high);

    fillbuf(demod->input,filter->input.c,filter->ilen*sizeof(*filter->input.c));
    spindown(demod,filter->input.c,filter->ilen); // 2nd LO
    execute_filter(filter);

    demod->snr = fm_snr(demod,filter->output.c,filter->olen);
    if(demod->snr > 2){
      int n;
      float avg = 0;
      extern float Misc;
      float const ampl = Misc * demod->amplitude;
      assert(filter->olen == afilter->ilen);
      for(n=0; n<filter->olen; n++){
	complex float samp = filter->output.c[n];
	if(cabsf(samp) > ampl){
	  avg += lastaudio = plfilter->input.r[n] = afilter->input.r[n] = carg(samp * state);
	  state = conjf(samp);
	} else {
	  plfilter->input.r[n] = afilter->input.r[n] = lastaudio;
	}
      }
      avg /= filter->olen;
      demod->foffset = dsamprate  * avg * M_1_2PI;

      // Find peak deviation, scale for output
      float pdev = 0;
      for(n=0; n < afilter->ilen; n++){
	if(fabsf(afilter->input.r[n] - avg) > pdev)
	  pdev = fabsf(afilter->input.r[n] - avg);
      }
      demod->pdeviation = dsamprate * pdev * M_1_2PI;
    } else {
      // Squelch is closed
      memset(afilter->input.r,0,afilter->ilen*sizeof(*afilter->input.r));
      memset(plfilter->input.r,0,plfilter->ilen*sizeof(*plfilter->input.r));
    }
    execute_filter(afilter);
    send_mono_audio(afilter->output.r,afilter->olen);

    // Determine PL tone frequency
    execute_filter(plfilter);
    double avgpl = 0;
    int n;
    for(n = 1;n < plfilter->olen;n++)
      avgpl += cargf(conjf(plfilter->output.c[n-1]) * plfilter->output.c[n]);
    avgpl /= (plfilter->olen-1);
    if(avgpl != 0)
      demod->plfreq = PL_samprate * avgpl * M_1_2PI;
  }
  delete_filter(plfilter);
  delete_filter(afilter);
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}

