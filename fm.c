// $Id: fm.c,v 1.43 2017/10/17 07:02:04 karn Exp karn $
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
  complex float state = 1; // Arbitrary choice; zero would cause first audio sample to be NAN
  float const dsamprate = demod->samprate / demod->decimate; // Decimated (output) sample rate
  demod->pdeviation = 0;
  demod->foffset = 0;
  demod->flags |= MONO; // Only mono for now
  demod->gain = NAN; // We don't use this, turn it off on the display

  // Create predetection filter, leaving response momentarily empty
  struct filter * const filter = create_filter(demod->L,demod->M,NULL,demod->decimate,COMPLEX,COMPLEX);
  demod->filter = filter;
  set_filter(demod->filter,dsamprate,demod->low,demod->high,demod->kaiser_beta);

  // Set up audio baseband filter
  // Response is high pass with 300 Hz corner, then -6 dB/octave post-emphasis since demod is FM
  // and modulation is probably PM (indirect FM)
  int const AL = demod->L / demod->decimate;
  int const AM = (demod->M - 1) / demod->decimate + 1;
  int const AN = AL + AM - 1;

  // Low pass filter to isolate PL tone at low sample rate
  int const PL_decimate = 32; // 48 kHz in, 1500 Hz out
  float const PL_samprate = dsamprate / PL_decimate;
  int const PL_L = AL / PL_decimate;
  int const PL_M = (AM - 1) / PL_decimate + 1;
  int const PL_N = PL_L + PL_M - 1;
  complex float * const plresponse = fftwf_alloc_complex(PL_N/2+1);
  // Filter gain for both filters
  // the '3' was set empirically, to make the audio levels sound comparable
  float const filter_gain = 3./AN;
  {
    memset(plresponse,0,(PL_N/2+1)*sizeof(*plresponse));
    // Positive frequencies only
    for(int j=0;j<=PL_N/2;j++){
      float const f = (float)j * dsamprate / AN; // frequencies are relative to INPUT sampling rate
      if(f > 0 && f < 300)
	plresponse[j] = filter_gain;
    }
  }
  window_rfilter(PL_L,PL_M,plresponse,2.0); // What's the optimum Kaiser window beta here?
  struct filter * const plfilter = create_filter(AL,AM,plresponse,PL_decimate,REAL,REAL);

  // Set up long FFT to which we feed the PL tone for frequency analysis
  int const pl_fft_size = (1 << 19) / PL_decimate;
  float * const pl_input = fftwf_alloc_real(pl_fft_size);
  complex float * const pl_spectrum = fftwf_alloc_complex(pl_fft_size/2+1);
  fftwf_plan pl_plan = fftwf_plan_dft_r2c_1d(pl_fft_size,pl_input,pl_spectrum,FFTW_ESTIMATE);

  // Voice filter, unless FLAT mode is selected
  struct filter *afilter = NULL;
  if(!(demod->flags & FLAT)){
    complex float * const aresponse = fftwf_alloc_complex(AN/2+1);
    memset(aresponse,0,(AN/2+1) * sizeof(*aresponse));
    for(int j=0;j<=AN/2;j++){
      float const f = (float)j * dsamprate / AN;
      if(f >= 300 && f <= 6000)
	aresponse[j] = filter_gain * 300./f; // -6 dB/octave de-emphasis to handle PM (indirect FM) transmission
    }
    // Window scaling for REAL input, REAL output
    window_rfilter(AL,AM,aresponse,demod->kaiser_beta);
    afilter = create_filter_slave(plfilter,aresponse,1); // Real input, real output, same sample rate
  }
  float lastaudio = 0; // state for impulse noise removal
  int fft_ptr = 0;

  while(!demod->terminate){
    fillbuf(demod,filter->input.c,filter->ilen);
    spindown(demod,filter->input.c); // 2nd LO
    demod->if_power = cpower(filter->input.c,filter->ilen);
    execute_filter(filter);
    // Compute bb_power below along with average amplitude to save time
    //    demod->bb_power = cpower(filter->output.c,filter->olen);
    float const n0 = compute_n0(demod);
    if(isnan(demod->n0))
      demod->n0 = n0; // handle startup transient
    else
      demod->n0 += .01 * (n0 - demod->n0);

    // Constant gain used by FM only; automatically adjusted by AGC in linear modes
    // We do this in the loop because BW can change
    float const gain = (demod->headroom *  M_1_PI * dsamprate) / fabsf(demod->low - demod->high);

    // Find average amplitude
    // We also need average magnitude^2, but we have that from demod->bb_power
    // Approximate for SNR because magnitude has a chi-squared distribution with 2 degrees of freedom
    float avg_amp = 0;
    demod->bb_power = 0;
    for(int n=0;n<filter->olen;n++){
      float const t = cnrmf(filter->output.c[n]);
      demod->bb_power += t;
      avg_amp += sqrtf(t);        // magnitude
    }
    demod->bb_power /= filter->olen;
    avg_amp /= filter->olen;         // Average magnitude
    float const fm_variance = demod->bb_power - avg_amp*avg_amp;
    demod->snr = avg_amp*avg_amp/(2*fm_variance) - 1;
    if(demod->snr < 0)
      demod->snr = 0; // Force to -infinity dB

    if(demod->snr > 2){
      // Threshold extension by comparing sample amplitude to threshold
      // 0.55 is empirical constant, 0.5 to 0.6 seems to sound good
      // Square amplitudes are compared to avoid sqrt inside loop
      float const min_ampl = 0.55 * 0.55 * avg_amp * avg_amp;
      assert(filter->olen == plfilter->ilen);
      // Actual FM demodulation, with impulse noise blanking
      float pdev_pos = 0;
      float pdev_neg = 0;
      float avg_f = 0;
      for(int n=0; n<filter->olen; n++){
	complex float const samp = filter->output.c[n];
	if(cnrmf(samp) > min_ampl){
	  lastaudio = plfilter->input.r[n] = cargf(samp * state);
	  state = conjf(samp);
	  // Keep track of peak deviation
	  if(n == 0)
	    pdev_pos = pdev_neg = lastaudio;
	  else if(lastaudio > pdev_pos)
	    pdev_pos = lastaudio;
	  else if(lastaudio < pdev_neg)
	    pdev_neg = lastaudio;
	} else {
	  plfilter->input.r[n] = lastaudio; // Replace unreliable sample with last good one
	}
	avg_f += lastaudio;
      }
      avg_f /= filter->olen;  // freq offset
      demod->foffset = dsamprate  * avg_f * M_1_2PI;

      // Find peak deviation allowing for frequency offset, scale for output
      pdev_pos -= avg_f;
      pdev_neg -= avg_f;
      demod->pdeviation = dsamprate * max(pdev_pos,-pdev_neg) * M_1_2PI;
    } else {
      // Squelch is closed
      memset(plfilter->input.r,0,plfilter->ilen*sizeof(*plfilter->input.r));
    }
    execute_filter(plfilter);
    if(afilter != NULL)
      execute_filter(afilter);

    // Determine PL tone frequency with a long FFT operating at the low PL filter sample rate
    assert(malloc_usable_size(pl_input) >= pl_fft_size * sizeof(float));
    assert(fft_ptr + plfilter->olen <= pl_fft_size);
    memcpy(pl_input+fft_ptr,plfilter->output.r,plfilter->olen*sizeof(*pl_input));
    fft_ptr = (fft_ptr + plfilter->olen) % pl_fft_size;

    // Let the filter tail leave after the squelch is closed, but don't send pure silence
    int silent = 1;
    float audio[plfilter->ilen];    
    if(afilter == NULL){
      for(int n=0; n < plfilter->ilen; n++){
	audio[n] = plfilter->input.r[n] * gain;
	if(audio[n] != 0)
	  silent = 0;
      }
    } else {
      assert(plfilter->ilen == afilter->olen);
      for(int n=0; n < afilter->olen; n++){
	audio[n] = afilter->output.r[n] * gain;
	if(audio[n] != 0)
	  silent = 0;
      }
    }
    if(!silent){

      send_mono_audio(demod->audio,audio,plfilter->ilen);

      // Determine PL tone, if any
      fftwf_execute(pl_plan);
      int peakbin = -1;      // Index of peak energy bin
      float peakenergy = 0;  // Energy in peak bin
      float totenergy = 0;   // Total energy, all bins
      assert(malloc_usable_size(pl_spectrum) >= pl_fft_size/2 * sizeof(complex float));
      for(int n=1;n<pl_fft_size/2;n++){ // skip DC
	float const energy = cnrmf(pl_spectrum[n]);
	totenergy += energy;
	if(energy > peakenergy){
	  peakenergy = energy;
	  peakbin = n;
	}
      }
      if(peakbin > 0 && peakenergy > 0.25 * totenergy ){
	// Standard PL tones range from 67.0 to 254.1 Hz; ignore out of range results
	// as they can be falsed by voice in the absence of a tone
	// Give a result only if the energy in the tone is at least quarter of the total (arbitrary)
	float const f = (float)peakbin * PL_samprate / pl_fft_size;
	if(f > 67 && f < 255)
	  demod->plfreq = f;
      } else
	demod->plfreq = NAN;
    }
  }
  fftwf_destroy_plan(pl_plan);
  fftwf_free(pl_input);
  fftwf_free(pl_spectrum);

  delete_filter(plfilter); // Must delete first
  if(afilter != NULL)
    delete_filter(afilter);
  delete_filter(filter);
  demod->filter = NULL;

  // Clear these to keep them from showing up with other demods
  demod->foffset = NAN;
  demod->pdeviation = NAN;
  demod->plfreq = NAN;
  demod->flags = 0;

  pthread_exit(NULL);
}

