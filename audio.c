// $Id: audio.c,v 1.28 2017/07/19 09:45:25 karn Exp karn $
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
int OPUS_stereo_read_fd;
int OPUS_stereo_write_fd;
int PCM_mono_read_fd;
int PCM_mono_write_fd;
int PCM_stereo_read_fd;
int PCM_stereo_write_fd;
float OPUS_blocktime = 20;
int OPUS_bitrate = 32000;
struct sockaddr_in BB_mcast_sockaddr;
pthread_t OPUS_stereo_thread;
pthread_t PCM_stereo_thread;
pthread_t PCM_mono_thread;

#define PCM_BUFSIZE 512        // 16-bit word count; must fit in Ethernet MTU


short const scaleclip(float x){
  if(x >= 1.0)
    return SHRT_MAX;
  else if(x <= -1.0)
    return SHRT_MIN;
  return (short)(SHRT_MAX * x);
}
  
int send_stereo_audio(complex float const *buffer,int size){
  if(OPUS_bitrate != 0)
    write(OPUS_stereo_write_fd,buffer,size * sizeof(*buffer));
  else
    write(PCM_stereo_write_fd,buffer,size * sizeof(*buffer));    
  return 0;
}

int send_mono_audio(float const *buffer,int size){
  if(OPUS_bitrate != 0){
    complex float obuf[size];
    int i;
    for(i=0;i<size;i++)
      obuf[i] = CMPLXF(buffer[i],buffer[i]);

    write(OPUS_stereo_write_fd,obuf,sizeof(obuf));
  } else
    write(PCM_mono_write_fd,buffer,size * sizeof(*buffer));    
  return 0;
}


void *stereo_opus_audio(void *arg){
  uint32_t timestamp = 0;
  uint16_t seq = 0;
  time_t tt = time(NULL);
  uint32_t const ssrc = tt & 0xffffffff;

  pthread_setname("opus");

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
  int const opus_blocksize = round(OPUS_blocktime * DAC_samprate / 1000.);
  complex float * const opusbuf = malloc(sizeof(complex float) * opus_blocksize);
  if(opusbuf == NULL){
    fprintf(stderr,"opus buffer malloc failed\n");
    return NULL;
  }

  int error;
  struct OpusEncoder * const Opus = opus_encoder_create(DAC_samprate,2,OPUS_APPLICATION_AUDIO,&error);
  if(Opus == NULL){
    fprintf(stderr,"opus_encoder_create failed, error %d\n",error);
    free(opusbuf);
    return NULL;
  }
  opus_encoder_ctl(Opus,OPUS_SET_BITRATE(OPUS_bitrate));
  opus_encoder_ctl(Opus,OPUS_SET_DTX(1));
  opus_encoder_ctl(Opus,OPUS_SET_LSB_DEPTH(16));

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
  message.msg_namelen = sizeof(BB_mcast_sockaddr);
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 2;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  while(1){
    if(fillbuf(OPUS_stereo_read_fd,opusbuf,sizeof(*opusbuf) * opus_blocksize) < 0){
      perror("stereo_opus_audio: pipe read error");
      break;
    }
    // Encoder accepts stereo, which we represent as complex, but it wants an array of floats
    int const dlen = opus_encode_float(Opus,(float *)opusbuf,opus_blocksize,data,sizeof(data));
    if(dlen < 0){
      fprintf(stderr,"opus encode error %d\n",dlen);
      continue;
    }
    if(dlen > 2){
      // Discontinuous transmission; don't send frames of 2 bytes or less,
      // but update timestamp so decoder will know how much was dropped
      rtp.seq = htons(seq++);
      rtp.timestamp = htonl(timestamp);
      iovec[1].iov_len = dlen; // Length varies
      sendmsg(Mcast_fd,&message,0);
    }
    timestamp += opus_blocksize;
  }
  opus_encoder_destroy(Opus);
  free(opusbuf);
  return NULL;
}
void *stereo_pcm_audio(void *arg){
  pthread_setname("stereo-pcm");
  uint32_t timestamp = 0;
  uint16_t seq = 0;
  time_t tt = time(NULL);
  uint32_t const ssrc = tt & 0xffffffff;

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
  message.msg_namelen = sizeof(BB_mcast_sockaddr);
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
  pthread_setname("mono-pcm");
  uint32_t timestamp = 0;
  uint16_t seq = 0;
  time_t tt = time(NULL);
  uint32_t const ssrc = tt & 0xffffffff;

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
  message.msg_namelen = sizeof(BB_mcast_sockaddr);
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
