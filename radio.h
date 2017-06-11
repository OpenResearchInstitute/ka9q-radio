// $Id: radio.h,v 1.21 2017/06/07 09:45:52 karn Exp karn $
#ifndef _RADIO_H
#define _RADIO_H 1

#include <complex.h>
#undef I
#include "command.h"

extern int ADC_samprate;


// Demodulator state block
struct demod {
  // Front end state
  double samprate;  // True A/D sample rate, assuming same TCXO as tuner
  double first_LO;  // UNcorrected local copy of frequency sent to front end tuner
  double calibrate; // Tuner TCXO calibration; - -> frequency low, + -> frequency high
                    // True first LO freq =  (1 + calibrate) * demod.first_LO
  uint8_t lna_gain;
  uint8_t mixer_gain;
  uint8_t if_gain;

  float DC_i,DC_q;   // Average DC offsets
  float power_i,power_q; // Average channel powers
  float igain;       // Amplitude gain to be applied to I channel to equalize I & Q, ideally 1
  float dotprod;     // smoothed dot product of I,Q for quadrature, ideally zero

  // Demod thread data
  int input;  // Input pipe fd
  pthread_t demod_thread;

  double dial_offset; // displayed dial frequency = carrier freq + dial_offset (useful for CW)

  // Second (software) local oscillator parameters
  complex double second_LO_phase;
  double second_LO; // True second LO frequency, including calibration
                    // Same as second_LO_phase step except when sweeping
                    // Provided because round trip through csincos/carg is less accurate
  complex double second_LO_phase_step;  // exp(2*pi*j*second_LO/samprate)
  double second_LO_rate;                // for frequency sweeping
  complex double second_LO_phase_accel;

  int frequency_lock;

  // Pre-demod filter parameters
  struct filter *filter;
  int L;            // Signal samples in FFT buffer
  int M;            // Samples in filter impulse response
  int decimate;     // Input/output sample rate decimation ratio
  float low;        // Edges of filter band
  float high;

  // Demodulator parameters
  enum mode mode;   // USB/LSB/FM/etc
  float amplitude;  // Amplitude (not power) of signal after filter
  float noise;      // Minimum amplitude for SNR estimates (experimental)
  float snr;        // Estimated signal-to-noise ratio
  float gain;       // Current audio gain (linear modes only)
  float foffset;    // Frequency offset (FM)
  float pdeviation; // Peak frequency deviation (FM)
};
extern struct demod Demod;
extern int Demod_sock;

extern const float Headroom; // Audio headroom ratio

const int LO2_in_range(const struct demod *demod,const double f);
const double get_freq(const struct demod *);
double set_freq(struct demod *,const double,const int);
double set_first_LO(struct demod *demod,const double first_LO,const int);
const double get_first_LO(const struct demod *demod);
double set_second_LO(struct demod *demod,const double second_LO,const int);
const double get_second_LO(const struct demod *demod,const int);
double set_second_LO_rate(struct demod *demod,const double second_LO_rate,const int);
const double get_exact_samprate(const struct demod *);
const int get_filter(const struct demod *demod,float *low,float *high);
int set_filter(struct demod *demod,const float low,const float high);

int set_mode(struct demod *demod,const enum mode mode);
int set_cal(struct demod *,const double);
const double get_cal(const struct demod *);
int spindown(struct demod *demod,complex float *data,const int len);
void closedown(const int a);
void proc_samples(struct demod *,const short *,const int);

// Thread entry points
void *fcd_command(void *);
void *display(void *);


void *demod_fm(void *);
void *demod_ssb(void *);
void *demod_iq(void *);
void *demod_cam(void *);
void *demod_am(void *);

#endif
