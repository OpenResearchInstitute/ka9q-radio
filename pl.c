// pltask to measure PL tone frequency with FFT
void *pltask(void *arg){
  pthread_setname("pl");
  struct demod *demod = (struct demod *)arg;

  assert(demod != NULL);

  // N, L and sample rate for audio master filter (usually 48 kHz)
  int const AN = (demod->filter.L + demod->filter.M - 1) / demod->filter.decimate;
  int const AL = demod->filter.L / demod->filter.decimate;
  float const dsamprate = (float)demod->input.samprate / demod->filter.decimate; // sample rate from FM demodulator

  // Pl slave filter parameters
  int PL_decimate = 1;
  if(AN >= 60)
    PL_decimate = AN / 30;

  float const PL_samprate = dsamprate / PL_decimate;
  int const PL_N = AN / PL_decimate;
  int const PL_L = AL / PL_decimate;
  int const PL_M = PL_N - PL_L + 1;

  // Low pass filter with 300 Hz cut to pass only PL tones
  complex float * const plresponse = fftwf_alloc_complex(PL_N/2+1);
  assert(plresponse != NULL);
  float const filter_gain = 1;
  memset(plresponse,0,(PL_N/2+1)*sizeof(*plresponse));
  // Positive frequencies only
  for(int j=0;j<=PL_N/2;j++){
    float const f = (float)j * dsamprate / AN; // frequencies are relative to INPUT sampling rate
    if(f > 0 && f < 300)
      plresponse[j] = filter_gain;
  } 
  window_rfilter(PL_L,PL_M,plresponse,2.0); // What's the optimum Kaiser window beta here?
  struct filter_out * const pl_filter = create_filter_output(demod->audio_master,plresponse,PL_decimate,REAL);

  // Set up long FFT to which we feed the PL tone for frequency analysis
  // PL analyzer sample rate = 48 kHz / 32 = 1500 Hz
  // FFT blocksize = 512k / 32 = 16k
  // i.e., one FFT buffer every 16k / 1500 = 10.92 sec, which gives < 0.1 Hz resolution
  int const pl_fft_size = (1 << 19) / PL_decimate;
  float * const pl_input = fftwf_alloc_real(pl_fft_size);
  complex float * const pl_spectrum = fftwf_alloc_complex(pl_fft_size/2+1);
  fftwf_plan pl_plan = fftwf_plan_dft_r2c_1d(pl_fft_size,pl_input,pl_spectrum,FFTW_ESTIMATE);
  assert(pl_plan != NULL);

  int fft_ptr = 0;
  int last_fft = 0;
  while(1){
    execute_filter_output(pl_filter,0);
 
    // Determine PL tone frequency with a long FFT operating at the low PL filter sample rate
    int remain = pl_filter->olen;
    last_fft += remain;
    float *data = pl_filter->output.r;
    while(remain != 0){
      int chunk = min(remain,pl_fft_size - fft_ptr);
      assert(malloc_usable_size(pl_input) >= sizeof(*pl_input) * (fft_ptr + chunk));
      memcpy(pl_input+fft_ptr,data,sizeof(*data) * chunk);
      fft_ptr += chunk;
      data += chunk;
      remain -= chunk;
      if(fft_ptr >= pl_fft_size)
	fft_ptr -= pl_fft_size;
    }
    // Execute only periodically
    if(last_fft >= 512){ // 512 / 1500 Hz = 0.34 seconds
      last_fft = 0;

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
      // Standard PL tones range from 67.0 to 254.1 Hz; ignore out of range results
      // as they can be falsed by voice in the absence of a tone
      // Give a result only if the energy in the tone exceeds an arbitrary fraction of the total
      if(peakbin > 0 && peakenergy > 0.01 * totenergy){
	float const f = (float)peakbin * PL_samprate / pl_fft_size;
	if(f > 67 && f < 255)
	  demod->sig.plfreq = f;
      } else
	demod->sig.plfreq = NAN;
    }
  }
}
