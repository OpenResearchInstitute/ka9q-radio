// $Id: audio.h,v 1.14 2017/06/21 09:08:40 karn Exp karn $
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
int setup_audio(void);

extern char BB_mcast_address_text[];
extern int Mcast_dest_port;
extern int DAC_samprate;
extern int OPUS_bitrate;
extern float OPUS_blocktime;
extern struct sockaddr_in BB_mcast_sockaddr;
extern int Mcast_fd;
extern int Verbose;


#endif


