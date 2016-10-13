// $Id$
#ifndef _RADIO_H
#define _RADIO_H 1

#include <complex.h>
#include "command.h"


// Demodulator state block
struct demod {
  enum mode mode;
  int samprate;     // Nominal sample rate of associated SDR front end
  int L;            // Signal samples in FFT buffer
  int M;            // Samples in filter impulse response
  double max_IF;
  double min_IF;
  double calibrate; // Local cop of tuner calibration; - -> frequency low, + -> frequency high
  double first_LO;  // Local copy of frequency sent to front end tuner
  double first_LO_err; // Fraction of hz LO is different from desired
  double second_LO; // Same as second_LO_phase step except when sweeping
                    // Provided because round trip through csincos/carg is less accurate
  double second_LO_rate;
  complex double second_LO_phase;
  complex double second_LO_phase_step;  // exp(2*pi*j*second_LO/samprate)
  complex double second_LO_phase_accel; // for frequency sweeping
  float energy;    // Input energy, current frame
  int low;          // Lower limit of passband in bins, modulo N (may be negative)
  int high;         // Upper limit of passband in bins, modulo N (may be negative)
  int decimate;     // Decimation ratio in frequency domain when filtering
  complex float *response; // Frequency response of pre-demod filter, set up by set_mode()
  struct filter *filter; // Pre-demodulation filter, set up by demod task using response
  complex float fmstate;    // FM demodulator state
  float snr;        // Estimated signal-to-noise ratio (FM only)
  float amplitude; // Amplitude (not power) of signal after filter
  int hangmax;      // How long for AGC to hang (clamp) after signal increases
  int hangtime;     // block times until AGC can increase again
  float gain;       // Current audio gain (linear modes only)
  float agcratio;   // Gain increase per block when not clamped
};
extern struct demod Demod;

double set_first_LO(double first_LO,int);
double set_second_LO(double second_LO,int);
double get_second_LO(void);
double set_second_LO_rate(double second_LO_rate,int);

int set_mode(enum mode mode);
int set_cal(double);
int ssb_agc();
int spindown(complex float *data,int len);
void closedown(int a);

// Thread entry points
void *fcd_command(void *);
void *dial(void *);     // Read tuning dial, set frequency
void *contour(void *);
void *display(void *);
void *demod_loop(void *);

#endif
