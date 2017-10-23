// $Id: audio.h,v 1.28 2017/09/26 18:12:12 karn Exp karn $
// Variables and structures for KA9Q SDR receiver audio subsystem
// Copyright 2017 Phil Karn, KA9Q

#ifndef _AUDIO_H
#define _AUDIO_H 1

#include <sys/types.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>
#include <portaudio.h>

struct audio {
  int samprate;       // Audio D/A sample rate (usually decimated from SDR A/D)

  // Audio buffer written to by demodulator, read by audio output routines
  pthread_mutex_t buffer_mutex;
  pthread_cond_t buffer_cond;
#define AUD_BUFSIZE (1<<17)
  float audiobuffer[AUD_BUFSIZE];
  int write_ptr;
  int read_ptr;       // for portaudio

  // Local audio output
  char localdev[128]; // Name of local audio device, if any
  PaStream *Pa_Stream; // local audio stream control block

  // File logging
  char *filename;
  FILE *stream;       // File pointer to output stream
  
  // RTP network streaming
  char audio_mcast_address_text[256];
  int audio_mcast_fd; // File descriptor for multicast output
  unsigned long long audio_packets;
  float bitrate;      // Average recent bitrate

  int rtp_pcm;        // Enable RTP pcm
  pthread_t pcm_thread;  // Uncompressed 16-bit linear PCM RTP sender

  // Opus encoder parameters
  // Opus only takes stereo because it compresses mono-as-stereo very efficiently
  pthread_t opus_stereo_thread;
  struct OpusEncoder *opus;
  float opus_blocktime;
  int opus_bitrate;
  int opus_dtx; // Discontinuous transmission (saves bandwidth)
};

extern struct audio Audio;

int send_mono_audio(struct audio *,float const *,int);
int send_stereo_audio(struct audio *,float const *,int);
int setup_audio(struct audio *);
void audio_cleanup(void *);

extern int DAC_samprate;
extern int Verbose;


#endif


