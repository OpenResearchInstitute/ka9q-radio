// $Id: audio.h,v 1.7 2017/06/12 18:20:27 karn Exp karn $
#ifndef _AUDIO_H
#define _AUDIO_H 1

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

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

extern struct sockaddr_storage PCM_mcast_sockaddr;
extern struct sockaddr_storage OPUS_mcast_sockaddr;


#endif


