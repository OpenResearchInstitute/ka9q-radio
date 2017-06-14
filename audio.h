// $Id: audio.h,v 1.8 2017/06/13 02:51:32 karn Exp karn $
#ifndef _AUDIO_H
#define _AUDIO_H 1

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

void *audio_thread(void *);
pthread_t Audio_thread;
int send_mono_audio(float *,int);
int send_stereo_audio(complex float *,int);
int setup_audio(float,int);

extern char *OPUS_mcast_address_text;
extern char *PCM_mcast_address_text;
extern int Mcast_dest_port;
extern int DAC_samprate;
extern complex float *Opusbuf;
extern int OPUS_bitrate;
extern int OPUS_blocksize;

extern struct sockaddr_storage PCM_mcast_sockaddr;
extern struct sockaddr_storage OPUS_mcast_sockaddr;
int Mcast_fd;

#endif


