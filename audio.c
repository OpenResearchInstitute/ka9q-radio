// $Id: audio.c,v 1.25 2017/06/21 09:26:44 karn Exp karn $
// Multicast PCM audio
#define _GNU_SOURCE 1
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <complex.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <complex.h>
#undef I
#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <opus/opus.h>

#include "rtp.h"
#include "dsp.h"
#include "audio.h"

int Mcast_fd;

struct sockaddr_in BB_mcast_sockaddr;
#define PCM_BUFSIZE 512        // 16-bit word count; must fit in Ethernet MTU


inline short scaleclip(float x){
  if(x >= 1.0)
    return SHRT_MAX;
  else if(x <= -1.0)
    return SHRT_MIN;
  return (short)(SHRT_MAX * x);
}
  
int OPUS_stereo_read_fd;
int OPUS_stereo_write_fd;
int PCM_mono_read_fd;
int PCM_mono_write_fd;
int PCM_stereo_read_fd;
int PCM_stereo_write_fd;




int send_stereo_audio(complex float *buffer,int size){
  if(OPUS_bitrate != 0)
    write(OPUS_stereo_write_fd,buffer,size * sizeof(complex float));
  else
    write(PCM_stereo_write_fd,buffer,size * sizeof(complex float));    
  return 0;
}
int send_mono_audio(float *buffer,int size){
  if(OPUS_bitrate != 0){
    complex float obuf[size];
    int i;
    for(i=0;i<size;i++)
      obuf[i] = CMPLXF(buffer[i],buffer[i]);

    write(OPUS_stereo_write_fd,obuf,size * sizeof(complex float));
  } else
    write(PCM_mono_write_fd,buffer,size * sizeof(float));    
  return 0;
}


float OPUS_blocktime;
int OPUS_bitrate;
int OPUS_blocksize;

void *stereo_opus_audio(void *arg){
  uint32_t timestamp = 0;
  uint16_t seq = 0;
  uint32_t ssrc;
  time_t tt = time(NULL);
  ssrc = tt & 0xffffffff;

  pthread_setname_np(pthread_self(),"opus");
  // Must correspond to 2.5, 5, 10, 20, 40, 60 ms
  // i.e., 120, 240, 480, 960, 1920, 2880 samples @ 48 kHz
  // opus 1.2 also supports 80, 100 and 120 ms
  if(OPUS_blocktime != 2.5 && OPUS_blocktime != 5
     && OPUS_blocktime != 10 && OPUS_blocktime != 20
     && OPUS_blocktime != 40 && OPUS_blocktime != 60
     && OPUS_blocktime != 80 && OPUS_blocktime != 100
     && OPUS_blocktime != 120){
    fprintf(stderr,"opus block time must be 2.5/5/10/20/40/60/80/100/120 ms\n");
    fprintf(stderr,"80/100/120 supported only on opus 1.2 and later\n");
    return NULL;
  }
  OPUS_blocksize = round(OPUS_blocktime * DAC_samprate / 1000.);

  int error;
  struct OpusEncoder *Opus = opus_encoder_create(DAC_samprate,2,OPUS_APPLICATION_AUDIO,&error);
  if(Opus == NULL){
    fprintf(stderr,"opus_encoder_create failed, error %d\n",error);
    return NULL;
  }
  complex float *opusbuf = malloc(sizeof(complex float) * OPUS_blocksize);
  if(opusbuf == NULL){
    fprintf(stderr,"opus buffer malloc failed\n");
    return NULL;
  }
  opus_encoder_ctl(Opus,OPUS_SET_BITRATE(OPUS_bitrate));

  struct rtp_header rtp;
  rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
  rtp.ssrc = htonl(ssrc);
  rtp.mpt = 20;         // arbitrary choice

  struct iovec iovec[2];
  unsigned char data[2048]; // Larger than Ethernet MTU
  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = data;
  
  struct msghdr message;
  message.msg_name = &BB_mcast_sockaddr;
  // OSX is very finicky about the size of the sockaddr structure; Linux is not
  message.msg_namelen = sizeof(struct sockaddr_in);
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 2;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  while(1){
    if(fillbuf(OPUS_stereo_read_fd,opusbuf,sizeof(complex float) * OPUS_blocksize) < 0){
      perror("stereo_opus_audio: pipe read error");
      break;
    }
    
    int dlen;
    dlen = opus_encode_float(Opus,(float *)opusbuf,OPUS_blocksize,data,sizeof(data));
    if(dlen < 0){
      fprintf(stderr,"opus encode error %d\n",dlen);
      continue;
    }
    rtp.seq = htons(seq++);
    rtp.timestamp = htonl(timestamp);
    timestamp += OPUS_blocksize;
    iovec[1].iov_len = dlen; // Length varies
    sendmsg(Mcast_fd,&message,0);
  }
  return NULL;
}
void *stereo_pcm_audio(void *arg){
  uint32_t timestamp = 0;
  uint16_t seq = 0;
  uint32_t ssrc;
  time_t tt = time(NULL);
  ssrc = tt & 0xffffffff;

  pthread_setname_np(pthread_self(),"stereo-pcm");
  struct rtp_header rtp;
  rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
  rtp.ssrc = htonl(ssrc);
  rtp.mpt = 10;         // 16 bit linear, big endian, stereo

  int16_t PCM_buf[PCM_BUFSIZE];

  struct iovec iovec[2];      
  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = PCM_buf;
  iovec[1].iov_len = sizeof(PCM_buf); // byte count - fixed
  
  struct msghdr message;      
  message.msg_name = &BB_mcast_sockaddr;
  // OSX is very finicky about the size of the sockaddr structure; Linux is not
  message.msg_namelen = sizeof(struct sockaddr_in);
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 2;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;
  
  while(1){
    complex float buffer[PCM_BUFSIZE/2];

    if(fillbuf(PCM_stereo_read_fd,buffer,sizeof(buffer)) < 0){
      perror("stereo_pcm_audio: pipe read error");
      break;
    }

    int i;
    for(i=0;i<PCM_BUFSIZE/2;i++){
      PCM_buf[2*i] = htons(scaleclip(crealf(buffer[i])));
      PCM_buf[2*i+1] = htons(scaleclip(cimagf(buffer[i])));
    }
    rtp.seq = htons(seq++);
    rtp.timestamp = htonl(timestamp);
    timestamp += PCM_BUFSIZE/2; // Increase by stereo sample count
    sendmsg(Mcast_fd,&message,0);
  }
  return NULL;
}

void *mono_pcm_audio(void *arg){
  uint32_t timestamp = 0;
  uint16_t seq = 0;
  uint32_t ssrc;
  time_t tt = time(NULL);
  ssrc = tt & 0xffffffff;

  pthread_setname_np(pthread_self(),"mono-pcm");
  struct rtp_header rtp;
  rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
  rtp.ssrc = htonl(ssrc);
  rtp.mpt = 11;         // 16 bit linear, big endian, mono

  int16_t PCM_buf[PCM_BUFSIZE];

  struct iovec iovec[2];      
  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = PCM_buf;
  iovec[1].iov_len = sizeof(PCM_buf); // byte count - fixed
  
  struct msghdr message;      
  message.msg_name = &BB_mcast_sockaddr;
  // OSX is very finicky about the size of the sockaddr structure; Linux is not
  message.msg_namelen = sizeof(struct sockaddr_in);
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 2;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;
  
  while(1){
    float buffer[PCM_BUFSIZE];

    if(fillbuf(PCM_mono_read_fd,buffer,sizeof(buffer)) < 0){
      perror("mono_pcm_audio: pipe read error");
      break;
    }

    int i;
    for(i=0;i<PCM_BUFSIZE;i++)
      PCM_buf[i] = htons(scaleclip(buffer[i]));

    rtp.seq = htons(seq++);
    rtp.timestamp = htonl(timestamp);
    timestamp += PCM_BUFSIZE; // Increase by stereo sample count
    sendmsg(Mcast_fd,&message,0);
  }
  return NULL;
}

pthread_t OPUS_stereo_thread;
pthread_t PCM_stereo_thread;
pthread_t PCM_mono_thread;

// Set up pipes to encoding/sending tasks and start them up
int setup_audio(){
  
  if(Verbose)
    fprintf(stderr,"%s\n",opus_get_version_string());
  int pipefd[2];

  pipe(pipefd);
  OPUS_stereo_read_fd = pipefd[0];
  OPUS_stereo_write_fd = pipefd[1];

  pipe(pipefd);
  PCM_stereo_read_fd = pipefd[0];
  PCM_stereo_write_fd = pipefd[1];

  pipe(pipefd);
  PCM_mono_read_fd = pipefd[0];
  PCM_mono_write_fd = pipefd[1];

  pthread_create(&OPUS_stereo_thread,NULL,stereo_opus_audio,NULL);
  pthread_create(&PCM_stereo_thread,NULL,stereo_pcm_audio,NULL);
  pthread_create(&PCM_mono_thread,NULL,mono_pcm_audio,NULL);  

  return 0;
}
