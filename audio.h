// $Id: audio.h,v 1.20 2017/08/04 14:53:33 karn Exp karn $
// Variables and structures for KA9Q SDR receiver audio subsystem
// Copyright 2017 Phil Karn, KA9Q

#ifndef _AUDIO_H
#define _AUDIO_H 1

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

struct audio {
  int samprate;       // Audio D/A sample rate (usually decimated from SDR A/D)
  char audio_mcast_address_text[256];
  int audio_mcast_fd; // File descriptor for multicast output
  long long audio_packets;
  float bitrate;      // Average recent bitrate

  
  // Opus encoder parameters
  // Opus only takes stereo because it compresses mono-as-stereo very efficiently
  pthread_t opus_stereo_thread;
  struct OpusEncoder *opus;
  float opus_blocktime;
  int opus_bitrate;
  int opus_dtx; // Discontinuous transmission (saves bandwidth)
  int opus_stereo_write_fd;  // Write 16-bit stereo PCM here
  int opus_stereo_read_fd;   // Opus thread reads from this

  // Uncompressed 16-bit linear mono and stereo PCM parameters
  pthread_t pcm_mono_thread;
  int pcm_mono_write_fd;     // Write 16-bit mono PCM here
  int pcm_mono_read_fd;      // Mono PCM thread reads from this

  pthread_t pcm_stereo_thread;
  int pcm_stereo_write_fd;   // Write 16-bit stereo PCM here
  int pcm_stereo_read_fd;    // Stereo PCM thread reads from this
};

extern struct audio Audio;

int send_mono_audio(struct audio *,float const *,int);
int send_stereo_audio(struct audio *,complex float const *,int);
int setup_audio(struct audio *);

extern int DAC_samprate;
extern int Verbose;


#endif


