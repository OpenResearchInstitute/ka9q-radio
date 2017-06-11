// $Id: audio.h,v 1.5 2017/06/05 06:09:17 karn Exp karn $
#ifndef _AUDIO_H
#define _AUDIO_H 1

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <alsa/asoundlib.h>

struct audio {
  int stereo_input;
  int mono_input;
};

extern struct audio Audio;
extern int Audio_mono_sock;
extern int Audio_stereo_sock;
int audio_change_parms(unsigned samplerate,int channels,int L);
int audio_out_done();
void *audio_thread(void *);
void *stream_thread(void *);
pthread_t Audio_thread;
pthread_t Stream_thread;
extern int Stream_write,Stream_read;
int send_mono_audio(float *,int);
int send_stereo_audio(complex float *,int);
int setup_audio(void);

extern struct sockaddr PCM_mcast_sockaddr;


#endif


