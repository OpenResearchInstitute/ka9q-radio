// $Id: radio.h,v 1.46 2017/09/21 00:14:32 karn Exp karn $
#ifndef _RADIO_H
#define _RADIO_H 1

#include <stdint.h>
#include <pthread.h>
#include <unistd.h>
#include <complex.h>
#undef I

#include <sys/types.h>
#include <sys/socket.h>

#define CTLPORT 7300

// Modetab flags
#define CONJ 1      // Cross-conjugation of positive and negative frequencies, for ISB
#define FLAT 2      // No baseband filtering for FM
#define COHERENT 4  // Coherent carrier tracking
#define CAL 8       // Calibrate mode in coherent demod; adjust calibrate rather than frequency
#define SQUARE   16 // Square carrier in coherent loop (BPSK/suppressed carrier AM)

struct modetab {
  char name[16];
  void * (*demod)(void *); // Address of demodulator routine
  int flags;        // Special purpose flags, e.g., CONJ
  float shift;      // Audio frequency shift (mainly for CW/RTTY)
  float tunestep;   // Default tuning step
  float low;        // Lower edge of IF passband
  float high;       // Upper edge of IF passband
};

// Sent in each RTP packet right after header
// NB! because we just copy this into the network stream, it's important that the compiler
// not add any extra padding. To avoid this, the size must be a multiple of 8, the size of the double
struct status {
  double frequency;
  uint32_t samprate;
  uint8_t lna_gain;
  uint8_t mixer_gain;
  uint8_t if_gain;
  uint8_t unused; // pad to 16 bytes
};

// Demodulator state block

struct demod {
  // Global parameters

  // Input thread state
  pthread_t input_thread;
  int input_fd;      // Raw incoming I/Q data from multicast socket
  char iq_mcast_address_text[256];
  struct sockaddr input_source_address;
  struct sockaddr ctl_address;
  unsigned long long iq_packets; // Count of I/Q input packets received

  // I/Q correction parameters
  float DC_i,DC_q;       // Average DC offsets
  float sinphi;          // smoothed estimate of I/Q phase error
  float imbalance;       // Ratio of I power to Q power

  struct status requested_status; // The status we want the FCD to be in
  struct status status;           // Last status from FCD
  // 'status' is written by the input thread and read by set_first_LO, etc, so it's protected by a mutex
  pthread_mutex_t status_mutex;
  pthread_cond_t status_cond;     // Signalled whenever status changes

  // True A/D sample rate, assuming same TCXO as tuner
  // Set from I/Q packet header and calibrate parameter
  volatile double samprate;

  // corr_data and write_ptr are written by the input thread and read by fillbuf in the demod tasks,
  // so they're protected by mutexes. The buffer is *NOT* protected from overrun, so the reader must keep up
#define DATASIZE 65536 // Strongly recommend a power of 2 for efficiency
  complex float *corr_data;       // Circular buffer of corrected I/Q data from input thread to demod thread
  int write_ptr;                  // 0 to DATASIZE-1
  int read_ptr;                   // 0 to DATASIZE-1
  pthread_mutex_t data_mutex;     // Protects corr_data and write_ptr
  pthread_cond_t data_cond;       // Signalled whenever data is written and write_ptr updated

  // Demodulator thread data
  pthread_t demod_thread;
  void * (*demod)(void *);        // Entry point to demodulator
  char mode[16];                  // printable mode name
  int terminate;                  // set to 1 by set_mode() to request graceful termination
  int flags;                      // Special flags to demodulator
  double start_freq;              // Initial frequency to set at startup

  // Tuning parameters
  int ctl_fd;                     // File descriptor for controlling SDR frequency and gaim

  // The tuner and A/D converter are clocked from the same TCXO
  // Ratio is (1+calibrate)
  //     calibrate < 0 --> TCXO frequency low; calibrate > 0 --> TCXO frequency high
  // True first LO = (1 + calibrate) * demod.status.frequency
  // True A/D sample rate = (1 + calibrate) * demod.status.samprate
  double calibrate;

  // Limits on usable IF due to aliasing, filtering, etc
  // Less than or equal to +/- samprate/2
  float min_IF;
  float max_IF;

  double frequency;     // Nominal (dial) frequency
  double doppler;       // Open-loop doppler correction from satellite tracking program
                        // To be handled by a separate spindown, not in radio.c


  // Second LO parameters
  complex double second_LO_phasor; // Second LO phasor
  double second_LO;     // True second LO frequency, including calibration
                        // Provided because round trip through csincos/carg is less accurate
  complex double second_LO_phasor_step;  // LO step phasor = csincos(2*pi*second_LO/samprate)

  double shift;         // frequency shift after demodulation (for CW,DSB)
  complex double shift_phasor;
  complex double shift_phasor_step;

  int frequency_lock; // inhibits tuning of RF and LO & IF tuning operate in lockstep
  int tunestep;       // User interface cursor location, log10(); e.g., 3 -> thousands

  // Experimental notch filter
  struct notchfilter *nf;

  // Zero IF pre-demod filter params
  struct filter *filter;
  int L;            // Signal samples in FFT buffer
  int M;            // Samples in filter impulse response
  int interpolate;  // Input sample ratio multiplier, should be power of 2
  int decimate;     // output sample rate divisor, should be power of 2
  float low;        // Edges of filter band
  float high;
  // Window shape factor for Kaiser window
  // Transition region is approx sqrt(1+Beta^2)
  // 0 => rectangular window; increasing values widens main lobe and decreases ripple
  float kaiser_beta;

  // Demodulator parameters
  float if_power;   // Average power of signal before filter
  float bb_power;   // Average power of signal after filter
  float n0;         // Noise spectral density esimate (experimemtal)
  float snr;        // Estimated signal-to-noise ratio (only some demodulators)
  float gain;       // Audio gain
  float foffset;    // Frequency offset (FM, coherent AM, cal, dsb)
  float pdeviation; // Peak frequency deviation (FM)
  float cphase;     // Carrier phase change (DSB/PSK)
  float plfreq;     // PL tone frequency (FM);
  float headroom;   // Audio level headroom
  float hangtime;   // SSB AGC hang time, seconds
  float recovery_rate; // SSB AGC recovery rate, db/sec

  struct audio *audio; // Link to audio output system
};
extern char Libdir[];
extern int Tunestep;
extern struct modetab Modes[];
extern int Nmodes;

int fillbuf(struct demod *,complex float *,const int);
int LO2_in_range(struct demod *,double f,int);
double set_freq(struct demod *,double,double);
double set_shift(struct demod *,double);
double set_first_LO(struct demod *,double);
const double get_first_LO(struct demod const *);
double set_second_LO(struct demod *,double);
double set_second_LO_rate(struct demod *,double,int);
int set_mode(struct demod *,const char *,int);
int set_cal(struct demod *,double);
complex float spindown(struct demod *,complex float const *);
void proc_samples(struct demod *,int16_t const *,int);
const float compute_n0(struct demod const *);


// Load mode definition table
int readmodes(char *);

// Save and load (most) receiver state
int savestate(struct demod *,char const *);
int loadstate(struct demod *,char const *);

// Save and load calibration
int loadcal(struct demod *,char const *);
int savecal(struct demod *,char const *);


// Thread entry points
void *display(void *);
void *keyboard(void *);

// Demodulator thread entry points
void *demod_fm(void *);
void *demod_am(void *);
void *demod_linear(void *);

#endif
