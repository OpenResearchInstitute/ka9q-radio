// $Id: fm.c,v 1.26 2017/07/18 00:43:25 karn Exp karn $
// FM demodulation and squelch
#define _GNU_SOURCE 1
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>
#undef I

#include "dsp.h"
#include "filter.h"
#include "radio.h"
#include "audio.h"


void *demod_fm(void *arg){
  pthread_setname("fm");
  assert(arg != NULL);
  struct demod * const demod = arg;  
  complex float state = 0;

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

  // Low pass filter to isolate PL tone at low sample rate
  const int PL_decimate = 32; // 48 kHz in, 1500 Hz out
  const int PL_samprate = dsamprate / PL_decimate;
  const int PL_L = AL / PL_decimate;
  const int PL_M = (AM - 1) / PL_decimate + 1;
  const int PL_N = PL_L + PL_M - 1;
  complex float * const plresponse = fftwf_alloc_complex(PL_N/2+1);
  memset(plresponse,0,(PL_N/2+1)*sizeof(*plresponse));
  // Positive frequencies only
  // Gain is still 1./AN
  for(j=0;j<=PL_N/2;j++){
    float const f = (float)j * dsamprate / AN; // frequencies are relative to INPUT sampling rate
    if(f > 0 && f < 300)
      plresponse[j] = gain;
  }
  window_rfilter(PL_L,PL_M,plresponse,2.0); // What's the optimum Kaiser window beta here?
  struct filter * const plfilter = create_filter_slave(afilter,plresponse,PL_decimate);

  // Set up long FFT to which we feed the PL tone for frequency analysis
  int const pl_fft_size = 524288 / PL_decimate;
  float * const pl_input = fftwf_alloc_real(pl_fft_size);
  complex float * const pl_spectrum = fftwf_alloc_complex(pl_fft_size/2+1);
  fftwf_plan pl_plan = fftwf_plan_dft_r2c_1d(pl_fft_size,pl_input,pl_spectrum,FFTW_ESTIMATE);

  float lastaudio = 0; // state for impulse noise removal

  while(!demod->terminate){
    // Constant gain used by FM only; automatically adjusted by AGC in linear modes
    // We do this in the loop because BW can change
    demod->gain = (Headroom *  M_1_PI * dsamprate) / fabsf(demod->low - demod->high);

    fillbuf(demod->input,filter->input.c,filter->ilen*sizeof(*filter->input.c));
    spindown(demod,filter->input.c,filter->ilen); // 2nd LO
    execute_filter(filter);

    // Find average magnitude and magnitude^2
    // Approximate for SNR because magnitude has a chi-squared distribution with 2 degrees of freedom
    float avg_squares = 0;
    float avg = 0;
    int n;
    for(n=0;n<filter->olen;n++){
      float const magsq = cnrmf(filter->output.c[n]); // I^2 + Q^2
      avg_squares += magsq;
      avg += sqrtf(magsq);            // magnitude
    }
    avg_squares /= filter->olen; // Average square magnitude
    avg /= filter->olen;         // Average magnitude
    demod->amplitude = avg;
    float const fm_variance = avg_squares - avg*avg;
    demod->noise = sqrtf(fm_variance);
    demod->snr = avg*avg/(2*fm_variance) - 1;
    if(demod->snr > 2){
      float avg = 0;
#if 0
      extern float Misc;
      float const ampl = Misc * demod->amplitude;
#else
      float const ampl = 0.55 * demod->amplitude; // Empirical constant, 0.5 to 0.6 seems to sound good
#endif
      assert(filter->olen == afilter->ilen);
      // Actual FM demodulation, with optional impulse noise blanking
      float pdev_pos = 0;
      float pdev_neg = 0;
      int n;      
      for(n=0; n<filter->olen; n++){
	complex float samp = filter->output.c[n];
	if(cabsf(samp) > ampl){
	  lastaudio = afilter->input.r[n] = carg(samp * state);
	  state = conjf(samp);
	  // Keep track of peak deviation
	  if(n == 0)
	    pdev_pos = pdev_neg = lastaudio;
	  else if(lastaudio > pdev_pos)
	    pdev_pos = lastaudio;
	  else if(lastaudio < pdev_neg)
	    pdev_neg = lastaudio;
	} else {
	  afilter->input.r[n] = lastaudio;
	}
	avg += lastaudio;
      }
      avg /= filter->olen;  // freq offset
      demod->foffset = dsamprate  * avg * M_1_2PI;

      // Find peak deviation allowing for frequency offset, scale for output
      pdev_pos -= avg;
      pdev_neg -= avg;
      demod->pdeviation = dsamprate * max(pdev_pos,-pdev_neg) * M_1_2PI;
    } else {
      // Squelch is closed
      memset(afilter->input.r,0,afilter->ilen*sizeof(*afilter->input.r));
    }
    execute_filter(afilter);
    execute_filter(plfilter); // Now slave to afilter
    // Determine PL tone frequency with a long FFT operating at the low PL filter sample rate
    assert(malloc_usable_size(pl_input) >= pl_fft_size * sizeof(float));
    memmove(pl_input,pl_input+plfilter->olen,(pl_fft_size-plfilter->olen)*sizeof(float));
    memcpy(pl_input+pl_fft_size-plfilter->olen,plfilter->output.r,plfilter->olen*sizeof(*pl_input));

    // Let the filter tail leave after the squelch is closed, but don't send pure silence
    for(n=0; n<afilter->olen; n++)
      if(afilter->output.r[n] != 0)
	break;

    if(n < afilter->olen){
      send_mono_audio(afilter->output.r,afilter->olen);
      fftwf_execute(pl_plan);
      int peakbin = -1;
      float peakenergy = 0;
      assert(malloc_usable_size(pl_spectrum) >= pl_fft_size/2 * sizeof(complex float));
      for(n=1;n<pl_fft_size/2;n++){
	if(cnrmf(pl_spectrum[n]) > peakenergy){
	  peakenergy = cnrmf(pl_spectrum[n]);
	  peakbin = n;
	}
      }
      if(peakbin >= 0)
	demod->plfreq = (float)peakbin * PL_samprate / pl_fft_size;
    }
  }
  fftwf_destroy_plan(pl_plan);
  fftwf_free(pl_input);
  fftwf_free(pl_spectrum);

  delete_filter(plfilter); // Must delete first
  delete_filter(afilter);
  delete_filter(filter);
  demod->filter = NULL;
  pthread_exit(NULL);
}

