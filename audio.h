// $Id: audio.h,v 1.16 2017/07/24 02:26:29 karn Exp karn $
#ifndef _AUDIO_H
#define _AUDIO_H 1

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

pthread_t Audio_thread;
int send_mono_audio(float const *,int);
int send_stereo_audio(complex float const *,int);
int setup_audio(void);

extern char BB_mcast_address_text[];
extern int DAC_samprate;
extern int OPUS_bitrate;
extern float OPUS_blocktime;
extern struct sockaddr_in BB_mcast_sockaddr;
extern int Mcast_fd;
extern int Verbose;


#endif


