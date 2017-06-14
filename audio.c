// $Id: audio.c,v 1.15 2017/06/13 09:27:17 karn Exp karn $
// Multicast PCM audio
#define _GNU_SOURCE 1
#include <assert.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <complex.h>
#include <stdint.h>
#include <time.h>
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

struct sockaddr_storage PCM_mcast_sockaddr;
uint32_t PCM_ssrc;
uint32_t PCM_timestamp = 0;
uint16_t PCM_seq = 0;
#define PCM_BUFSIZE 700        // 16-bit word count; must fit in Ethernet MTU
int16_t PCM_buf[PCM_BUFSIZE];
int PCM_writeptr = 0;


struct OpusEncoder *Opus;
struct sockaddr_storage OPUS_mcast_sockaddr;
const int Opusbitrate = 32000;
// Must correspond to 2.5, 5, 10, 20, 40, 60 ms
// i.e., 120, 240, 480, 960, 1920, 2880 samples @ 48 kHz
#define OPUSBUFSIZE 960
complex float Opusbuf[OPUSBUFSIZE];
int Opuswriteptr = 0;
uint32_t OPUS_ssrc;
uint32_t OPUS_timestamp = 0;
uint16_t OPUS_seq = 0;


// Send floating point linear PCM as OPUS compressed multicast
// size = # of stereo samples (half number of floats)
int send_stereo_opus(complex float *buffer,int size){
  int space,chunk;

  if(OPUS_mcast_sockaddr.ss_family != AF_INET && OPUS_mcast_sockaddr.ss_family != AF_INET6)
     return 0;
  while(size > 0){
    space = OPUSBUFSIZE - Opuswriteptr;
    chunk = min(space,size);
    memcpy(&Opusbuf[Opuswriteptr],buffer,chunk*sizeof(complex float));
    Opuswriteptr += chunk;
    buffer += chunk;
    size -= chunk;

    if(Opuswriteptr == OPUSBUFSIZE){
      // Full, send it
      
      int dlen;
      unsigned char data[2048]; // Larger than Ethernet MTU
      struct rtp_header rtp;
      struct iovec iovec[2];
      struct msghdr message;
      
      dlen = opus_encode_float(Opus,(float *)Opusbuf,OPUSBUFSIZE,data,sizeof(data));
      rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
      rtp.ssrc = htonl(OPUS_ssrc);
      rtp.mpt = 20;         // arbitrary choice
      rtp.seq = htons(OPUS_seq++);
      rtp.timestamp = htonl(OPUS_timestamp);
      OPUS_timestamp += OPUSBUFSIZE;
      
      iovec[0].iov_base = &rtp;
      iovec[0].iov_len = sizeof(rtp);
      iovec[1].iov_base = data;
      iovec[1].iov_len = dlen;
      
      message.msg_name = &OPUS_mcast_sockaddr;
      message.msg_namelen = sizeof(OPUS_mcast_sockaddr);
      message.msg_iov = &iovec[0];
      message.msg_iovlen = 2;
      message.msg_control = NULL;
      message.msg_controllen = 0;
      message.msg_flags = 0;
      sendmsg(Mcast_fd,&message,0);
      Opuswriteptr = 0;
    }
  }
  return 0;
}


int send_mono_opus(const float *buffer,int size){
  complex float cbuffer[size];
  int i;

  if(OPUS_mcast_sockaddr.ss_family != AF_INET && OPUS_mcast_sockaddr.ss_family != AF_INET6)
     return 0;

  for(i=0;i<size;i++)
    cbuffer[i] = CMPLXF(buffer[i],buffer[i]);

  send_stereo_opus(cbuffer,size);
  return 0;
}

int send_mono_audio(float *buffer,int size){
  send_mono_opus(buffer,size);

  if(PCM_mcast_sockaddr.ss_family != AF_INET && PCM_mcast_sockaddr.ss_family != AF_INET6)
     return 0;

  while(size > 0){
    int chunk = PCM_BUFSIZE - PCM_writeptr;
    chunk = min(chunk,size);
    int i;
    for(i=0;i<chunk;i++)
      PCM_buf[PCM_writeptr+i] = htons(CLIP(SHRT_MAX * buffer[i]));

    PCM_writeptr += chunk;
    buffer += chunk;
    size -= chunk;

    assert(PCM_writeptr <= PCM_BUFSIZE);
    if(PCM_writeptr == PCM_BUFSIZE){
      // Full, send it

      struct rtp_header rtp;      
      rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
      rtp.ssrc = htonl(PCM_ssrc);
      rtp.mpt = 11;         // 16 bit linear, big endian, mono
      rtp.seq = htons(PCM_seq++);
      rtp.timestamp = htonl(PCM_timestamp);
      PCM_timestamp += PCM_BUFSIZE; // Increase by mono sample count
      struct iovec iovec[2];      
      iovec[0].iov_base = &rtp;
      iovec[0].iov_len = sizeof(rtp);
      iovec[1].iov_base = PCM_buf;
      iovec[1].iov_len = PCM_BUFSIZE * 2; // byte count
      struct msghdr message;      
      message.msg_name = &PCM_mcast_sockaddr;
      message.msg_namelen = sizeof(PCM_mcast_sockaddr);
      message.msg_iov = &iovec[0];
      message.msg_iovlen = 2;
      message.msg_control = NULL;
      message.msg_controllen = 0;
      message.msg_flags = 0;
      sendmsg(Mcast_fd,&message,0);
      PCM_writeptr = 0;
    }
  }
  return 0;
}
int send_stereo_audio(complex float *buffer,int size){
  send_stereo_opus(buffer,size);

  if(PCM_mcast_sockaddr.ss_family != AF_INET && PCM_mcast_sockaddr.ss_family != AF_INET6)
     return 0;

  while(size > 0){
    int chunk = PCM_BUFSIZE - PCM_writeptr;
    chunk = min(chunk/2,size); // count of stereo samples (4 bytes, 2 16-bit words)
    int i;
    for(i=0;i<chunk;i++){
      PCM_buf[PCM_writeptr+2*i] = htons(CLIP(SHRT_MAX * crealf(buffer[i])));
      PCM_buf[PCM_writeptr+2*i+1] = htons(CLIP(SHRT_MAX * cimagf(buffer[i])));
    }
    PCM_writeptr += 2*chunk;
    buffer += chunk;
    size -= chunk;

    assert(PCM_writeptr <= PCM_BUFSIZE);
    if(PCM_writeptr == PCM_BUFSIZE){
      // Full, send it
      
      struct rtp_header rtp;
      rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
      rtp.ssrc = htonl(PCM_ssrc);
      rtp.mpt = 10;         // 16 bit linear, big endian, stereo
      rtp.seq = htons(PCM_seq++);
      rtp.timestamp = htonl(PCM_timestamp);
      PCM_timestamp += PCM_BUFSIZE/2; // Increase by stereo sample count
      struct iovec iovec[2];      
      iovec[0].iov_base = &rtp;
      iovec[0].iov_len = sizeof(rtp);
      iovec[1].iov_base = PCM_buf;
      iovec[1].iov_len = PCM_BUFSIZE * 2; // byte count
      struct msghdr message;      
      message.msg_name = &PCM_mcast_sockaddr;
      message.msg_namelen = sizeof(PCM_mcast_sockaddr);
      message.msg_iov = &iovec[0];
      message.msg_iovlen = 2;
      message.msg_control = NULL;
      message.msg_controllen = 0;
      message.msg_flags = 0;
      sendmsg(Mcast_fd,&message,0);
      PCM_writeptr = 0;
    }
  }
  return 0;
}


int setup_audio(int bitrate){
  time_t tt;
  time(&tt);
  PCM_ssrc = tt & 0xffffffff; // low 32 bits of clock time

  OPUS_ssrc = (tt+1) & 0xffffffff; // low 32 bits of clock time
  int error;
  Opus = opus_encoder_create(48000,2,OPUS_APPLICATION_AUDIO,&error);
  opus_encoder_ctl(Opus,OPUS_SET_BITRATE(bitrate));
  return 0;
}
