// $Id: audio.h,v 1.18 2017/08/02 02:29:17 karn Exp karn $
#ifndef _AUDIO_H
#define _AUDIO_H 1

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

struct audio {
  // Audio parameters
  float opus_blocktime;
  int opus_bitrate;
  char audio_mcast_address_text[256];
  int opus_stereo_write_fd;
  int opus_stereo_read_fd;
  int pcm_mono_read_fd;
  int pcm_mono_write_fd;
  int pcm_stereo_read_fd;
  int pcm_stereo_write_fd;
  struct OpusEncoder *opus;
  pthread_t opus_stereo_thread;
  pthread_t pcm_stereo_thread;
  pthread_t pcm_mono_thread;
  int audio_mcast_fd;
};

struct audio Audio;


int send_mono_audio(struct audio *,float const *,int);
int send_stereo_audio(struct audio *,complex float const *,int);
int setup_audio(struct audio *);

extern int DAC_samprate;
extern int Verbose;


#endif


