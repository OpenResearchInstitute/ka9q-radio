// $Id: radio.h,v 1.14 2017/05/31 22:27:19 karn Exp karn $
#ifndef _RADIO_H
#define _RADIO_H 1

#include <complex.h>
#undef I
#include "command.h"


// Demodulator state block
struct demod {
  enum mode mode;
  int samprate;     // Nominal sample rate of associated SDR front end
  int L;            // Signal samples in FFT buffer
  int M;            // Samples in filter impulse response
  double max_IF;
  double min_IF;
  double calibrate; // Tuner calibration; - -> frequency low, + -> frequency high
  double first_LO;  // Local copy of frequency sent to front end tuner, uncorrected
  double second_LO; // Same as second_LO_phase step except when sweeping
                    // Provided because round trip through csincos/carg is less accurate
  double second_LO_rate;
  float DC_i,DC_q;  // Average DC offsets
  float power_i,power_q; // Average channel powers
  float igain;       // Amplitude gain to be applied to I channel to equalize I & Q, ideally 1
  float sinphi;      // Sine of I/Q phase error, ideally zero
  uint8_t lna_gain;
  uint8_t mixer_gain;
  uint8_t if_gain;


  complex double second_LO_phase;
  complex double second_LO_phase_step;  // exp(2*pi*j*second_LO/samprate)
  complex double second_LO_phase_accel; // for frequency sweeping
  double dial_offset;
  int decimate;     // Decimation ratio in frequency domain when filtering
  struct filter *filter; // Pre-demodulation filter, set up by demod task using response
  float snr;        // Estimated signal-to-noise ratio (FM only)
  float amplitude; // Amplitude (not power) of signal after filter
  float noise;     // Minimum amplitude for SNR estimates (experimental)
  float gain;       // Current audio gain (linear modes only)
  float foffset;    // Frequency offset (FM)
  float pdeviation;
  int devhold;
  pthread_t demod_thread;
  int data_sock;
};
extern struct demod Demod;
extern int Demod_sock;

extern const float Headroom; // Audio headroom ratio

double get_freq(struct demod *);
double set_freq(struct demod *,double,int);
double set_first_LO(struct demod *demod,double first_LO,int);
double get_first_LO(struct demod *demod);
double set_second_LO(struct demod *demod,double second_LO,int);
double get_second_LO(struct demod *demod,int);
double set_second_LO_rate(struct demod *demod,double second_LO_rate,int);
double get_exact_samprate(struct demod *);

int set_mode(struct demod *demod,enum mode mode);
int set_cal(struct demod *,double);
double get_cal(struct demod *);
int spindown(struct demod *demod,complex float *data,int len);
void closedown(int a);
void proc_samples(struct demod *,short *,int);

// Thread entry points
void *fcd_command(void *);
void *display(void *);


void *demod_fm(void *);
void *demod_ssb(void *);
void *demod_iq(void *);
void *demod_cam(void *);
void *demod_am(void *);

#endif
