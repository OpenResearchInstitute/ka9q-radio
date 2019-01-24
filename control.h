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

#include <stdint.h>
#include <sys/socket.h>

#include "sdr.h"
#include "multicast.h"
#include "osc.h"

#define PKTSIZE 16384

// Control state block
// This began life as 'struct demod' in radio.h, but has gone its separate way
struct control {

  // Network between SDR and radio
  struct {
    char description[256]; // Free-form text

    struct sockaddr_storage metadata_source_address; // Source of SDR metadata
    struct sockaddr_storage metadata_dest_address;   // Dest of metadata (typically multicast)
    uint64_t metadata_packets;
    
    struct sockaddr_storage data_source_address; // Source of I/Q data
    struct sockaddr_storage data_dest_address;   // Dest of I/Q data (typically multicast)
    struct rtp_state rtp; // State of the I/Q RTP receiver
    uint64_t samples;    // Count of raw I/Q samples received
    int samprate;
    uint64_t commands;
  } input;

  // Front end hardware information
  struct {
    struct status status;           // Last status from FCD
    int direct_conversion;          // Avoid 0 Hz if set
    double calibration;
    // I/Q correction parameters
    float DC_i,DC_q;       // Average DC offsets
    float sinphi;          // smoothed estimate of I/Q phase error
    float imbalance;       // Ratio of I power to Q power
    float ad_level;        // A/D signal level, dBFS
    
    // Limits on usable IF due to aliasing, filtering, etc
    // Less than or equal to +/- samprate/2
    float min_IF;
    float max_IF;
  } sdr;
  
  // Radio parameters
  struct {
    int lock;       // When set, don't try to command tuner
    double freq;    // Desired carrier frequency
    double second_LO;
    double doppler;
    double doppler_rate;
    double shift;   // Post-demod frequency shift
    int step;       // Tuning column, log10(); e.g., 3 -> thousands (local to control)
    int item;       // Tuning entry index (local to control)
  } tune;

  // Radio IF pre-demod filter params
  struct {
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
    int isb;     // Independent sideband mode
  } filter;

  enum demod_type demod_type;            // Index into demodulator table (AM, FM, Linear)

  // Various radio options
  struct {
    int flat;    // Flat FM frequency response
    int env;     // Envelope detection in linear mode
    int pll;     // Linear mode PLL tracking of carrier
    int square;  // Squarer on PLL input
    float loop_bw;    // Loop bw (coherent modes)
    int agc;
  } opt;

  // Radio AGC (AM and linear modes)
  struct {
    float headroom;   // Audio level headroom, amplitude ratio
    float hangtime;   // AGC hang time, samples
    float recovery_rate; // AGC recovery rate, amplitude ratio/sample 
    float attack_rate;   // AGC attack rate, amplitude ratio/sample
    float gain;       // Audio gain, amplitude ratio
  } agc;

  // Radio signal levels & status
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
    int pll_lock;
  } sig;
  
  // Radio output
  struct {
    int samprate;       // Audio D/A sample rate (usually 48 kHz)
    // RTP network streaming
    struct rtp_state rtp;
    struct sockaddr_storage metadata_source_address; // Source of SDR metadata
    struct sockaddr_storage metadata_dest_address;   // Dest of metadata (typically multicast)
    uint64_t metadata_packets;
    
    struct sockaddr_storage data_source_address; // Source address of our data output
    struct sockaddr_storage data_dest_address;   // Dest of our data outputg (typically multicast)
    
    int channels;   // 1 = mono, 2 = stereo
    uint32_t command_tag; // Echoed in responses to commands
    float level;    // Output level
    uint64_t samples;
    uint64_t commands;
  } output;

  // Opus transcoder, if present (separate program)
  struct {
    struct sockaddr_storage source_address; // when opus codec is used
    struct sockaddr_storage dest_address; 
    uint32_t ssrc; 
    int ttl;
    int bitrate;
    int packets;
  } opus;
};

extern int Verbose;
extern int Mcast_ttl;

#endif
