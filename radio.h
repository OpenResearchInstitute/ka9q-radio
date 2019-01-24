// $Id: radio.h,v 1.97 2019/01/24 05:00:00 karn Exp karn $
// Internal structures and functions of the 'radio' program
// Nearly all internal state is in the 'demod' structure
// More than one can exist in the same program,
// but so far it seems easier to just run separate instances of the 'radio' program.
// Copyright 2018, Phil Karn, KA9Q
#ifndef _RADIO_H
#define _RADIO_H 1

#include <pthread.h>
#include <complex.h>
#undef I

#include <sys/socket.h>
#include <stdint.h>
#include <stdbool.h>

#include "modes.h"
#include "sdr.h"
#include "multicast.h"
#include "osc.h"

#define PKTSIZE 16384
// Incoming RTP packets
// This should probably be extracted into a more general RTP library
struct packet {
  struct packet *next;
  struct rtp_header rtp;
  unsigned char *data;
  int len;
  unsigned char content[PKTSIZE];
};

// Demodulator state block
struct demod {

  // Multicast network connection with SDR front end
  struct {
    char description[256]; // Free-form text
    int data_fd;           // Socket for raw incoming I/Q data
    int ctl_fd;            // Socket for commands to front end
    int status_fd;         // Socket for status from front end

    struct sockaddr_storage metadata_source_address; // Source of SDR metadata
    struct sockaddr_storage metadata_dest_address;   // Dest of metadata (typically multicast)
    uint64_t metadata_packets;
    
    struct sockaddr_storage data_source_address; // Source of I/Q data
    struct sockaddr_storage data_dest_address;   // Dest of I/Q data (typically multicast)
    struct rtp_state rtp; // State of the I/Q RTP receiver
    uint64_t samples;    // Count of raw I/Q samples received
    int samprate;
    uint32_t command_tag;  // Our tag for pending command to front end
    uint64_t commands;
  } input;

  // Front end hardware information (some of it is ignored by us, used only by control)
  struct {
    struct status status;           // Last status from FCD
    bool direct_conversion;          // Avoid 0 Hz if set
    
    // Limits on usable IF due to aliasing, filtering, etc
    // Less than or equal to +/- samprate/2
    float min_IF;
    float max_IF;

    float gain_factor;     // Multiply by incoming samples to scale by analog AGC settings

    // 'status' is written by the input thread and read by set_first_LO, etc, so it's protected by a mutex
    pthread_mutex_t status_mutex;
    pthread_cond_t status_cond;     // Signalled whenever status changes
  } sdr;
  
  // Tuning parameters
  struct {
    double freq;    // Desired carrier frequency
    double shift;   // Post-demod frequency shift
    double second_LO;
    double doppler;
    double doppler_rate;
  } tune;

  struct osc doppler;
  struct osc second_LO;
  struct osc shift;

  // Zero IF pre-demod filter params
  struct {
    struct filter_in *in;
    struct filter_out *out;
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
    float noise_bandwidth; // noise bandwidth relative to sample rate
    bool isb;     // Independent sideband mode
  } filter;

  // Protect demod_type
  pthread_mutex_t demod_mutex;
  pthread_cond_t demod_cond;
  enum demod_type demod_type;            // Index into demodulator table (AM, FM, Linear)

  struct {
    bool flat;    // Flat FM frequency response
    bool env;     // Envelope detection in linear mode
    bool pll;     // Linear mode PLL tracking of carrier
    bool square;  // Squarer on PLL input
    float loop_bw;    // Loop bw (coherent modes)
    bool agc;     // Automatic gain control enabled
  } opt;

  // AGC (AM and linear modes)
  struct {
    float headroom;   // Audio level headroom, amplitude ratio
    float hangtime;   // AGC hang time, samples
    float recovery_rate; // AGC recovery rate, amplitude ratio/sample 
    float attack_rate;   // AGC attack rate, amplitude ratio/sample
    float gain;       // Audio gain, amplitude ratio
  } agc;

  // Signal levels & status
  struct {
    float if_power;   // Input level, unity == 0dBFS, power ratio
    float bb_power;   // Average power of signal after filter, power ratio
    float n0;         // Noise spectral density esimate (experimemtal), power/Hz ratio
    float snr;        // Estimated signal-to-noise ratio (only some demodulators), power ratio
    float foffset;    // Frequency offset Hz (FM, coherent AM, dsb)
    float pdeviation; // Peak frequency deviation Hz (FM)
    float cphase;     // Carrier phase change radians (DSB/PSK)
    float plfreq;     // PL tone frequency Hz (FM)
    float lock_timer; // PLL lock timer
    bool pll_lock;    // PLL is locked
  } sig;
  
  // Output
  struct {
    int samprate;       // Audio D/A sample rate (usually 48 kHz)
    // RTP network streaming
    bool silent; // last packet was suppressed (used to generate RTP mark bit)
    struct rtp_state rtp;
    struct sockaddr_storage metadata_source_address; // Source of SDR metadata
    struct sockaddr_storage metadata_dest_address;   // Dest of metadata (typically multicast)
    uint64_t metadata_packets;
    
    struct sockaddr_storage data_source_address; // Source address of our data output
    struct sockaddr_storage data_dest_address;   // Dest of our data outputg (typically multicast)
    
    int data_fd;    // File descriptor for multicast output
    int rtcp_fd;    // File descriptor for RTP control protocol
    int status_fd;  // File descriptor for receiver status
    int ctl_fd;     // File descriptor for receiving user commands
    int channels;   // 1 = mono, 2 = stereo
    uint32_t command_tag; // Echoed in responses to commands
    float level;    // Output level
    uint64_t samples;
    uint64_t commands;
  } output;
};
extern char Libdir[];
extern int Verbose;

// Functions/methods to control a demod instance
int LO2_in_range(struct demod *,double f,int);
double get_freq(struct demod *);
double set_freq(struct demod *,double,double);
double get_shift(struct demod *);
double set_shift(struct demod *,double);
double get_first_LO(struct demod const *);
double set_first_LO(struct demod *,double);
double get_second_LO(struct demod *);
double set_second_LO(struct demod *,double);
double get_doppler(struct demod *);
double get_doppler_rate(struct demod *);
int set_doppler(struct demod *,double,double);
int preset_mode(struct demod *,const char *);

void *proc_samples(void *);
const float compute_n0(struct demod const *);

// Demodulator thread entry points
void *demod_fm(void *);
void *demod_linear(void *);

int send_mono_output(struct demod *,const float *,int);
int send_stereo_output(struct demod *,const float *,int);
void output_cleanup(void *);

extern int Mcast_ttl;

#endif
