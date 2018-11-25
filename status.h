enum status_type {
  EOL = 0,	  
  TYPE,
  INPUT_SOURCE_ADDRESS,
  INPUT_SOURCE_PORT,
  INPUT_DEST_ADDRESS,
  INPUT_DEST_PORT,
  INPUT_SSRC,
  INPUT_SAMPRATE,
  INPUT_PACKETS,
  INPUT_SAMPLES,
  INPUT_DROPS,
  INPUT_DUPES,

  OUTPUT_DEST_ADDRESS, // 12
  OUTPUT_DEST_PORT,
  OUTPUT_SSRC,
  OUTPUT_TTL,
  OUTPUT_SAMPRATE,
  OUTPUT_PACKETS,

  // Tuning
  CENTER_FREQUENCY, // 18
  INTERMEDIATE_FREQUENCY,
  SHIFT_FREQUENCY,
  DOPPLER_FREQUENCY,
  DOPPLER_FREQUENCY_RATE,

  // Hardware gains
  LNA_GAIN, // 23
  MIX_GAIN,
  IF_GAIN,

  // Filtering
  LOW_EDGE, // 26
  HIGH_EDGE,
  KAISER_BETA,
  FILTER_BLOCKSIZE,
  FILTER_FIR_LENGTH,

  // Signals
  IF_POWER, // 31
  BASEBAND_POWER,
  NOISE_DENSITY,

  // Demodulation
  RADIO_MODE, // printable string "usb", "lsb", etc
  DEMOD_MODE, // 1 = AM envelope, 2 = FM, 3 = linear // 35
  INDEPENDENT_SIDEBAND, // Linear
  DEMOD_SNR,       // FM, PLL linear
  DEMOD_GAIN,      // AM, Linear
  FREQ_OFFSET,     // FM, PLL linear

  PEAK_DEVIATION, // FM 40
  PL_TONE,        // FM
  
  PLL_LOCK,       // Linear PLL 42
  PLL_SQUARE,     // Linear PLL
  PLL_PHASE,      // Linear PLL

  OUTPUT_CHANNELS, // 1/2 in Linear, otherwise 1; 45

};
