// $Id: audio.c,v 1.13 2017/06/13 02:51:31 karn Exp karn $
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

struct audio Audio;

int Mcast_fd;

struct sockaddr_storage PCM_mcast_sockaddr;
uint32_t PCM_ssrc;
uint32_t PCM_timestamp = 0;
uint16_t PCM_seq = 0;

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

  while(size > 0){
    space = OPUSBUFSIZE - Opuswriteptr;
    chunk = min(space,size);
    memcpy(&Opusbuf[Opuswriteptr],buffer,chunk*sizeof(complex float));
    Opuswriteptr += chunk;
    buffer += chunk;
    size -= chunk;

    if(Opuswriteptr == OPUSBUFSIZE){
      // Full, send it
      
      unsigned char data[1024];
      int dlen;
      struct rtp_header rtp;
      struct iovec iovec[2];
      struct msghdr message;
      
      // We do assume a complex consists of a real followed by imaginary
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

  for(i=0;i<size;i++)
    cbuffer[i] = CMPLXF(buffer[i],buffer[i]);

  send_stereo_opus(cbuffer,size);
  return 0;
}

int send_mono_audio(float *buffer,int size){
  const int L = 512;
  int i,chunk;
  float *sp;

  send_mono_opus(buffer,size);

  sp = buffer;
  while(size > 0){
    struct rtp_header rtp;
    struct iovec iovec[2];
    struct msghdr message;
    signed short outsamps[L];

    chunk = min(size,L);

    for(i=0;i<chunk;i++)
      outsamps[i] = htons(CLIP(SHRT_MAX * sp[i])); // Scale, clip and byte swap
  
    rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
    rtp.ssrc = htonl(PCM_ssrc);
    rtp.mpt = 11;         // Mono linear 16-bit pcm 48 kHz
    rtp.seq = htons(PCM_seq++);
    rtp.timestamp = htonl(PCM_timestamp);
    PCM_timestamp += chunk;
    
    iovec[0].iov_base = &rtp;
    iovec[0].iov_len = sizeof(rtp);
    iovec[1].iov_base = outsamps;
    iovec[1].iov_len = chunk * sizeof(outsamps[0]); // 2 bytes/sample
    
    message.msg_name = &PCM_mcast_sockaddr;
    message.msg_namelen = sizeof(PCM_mcast_sockaddr);
    message.msg_iov = &iovec[0];
    message.msg_iovlen = 2;
    message.msg_control = NULL;
    message.msg_controllen = 0;
    message.msg_flags = 0;
    sendmsg(Mcast_fd,&message,0);
    
    size -= chunk;
    sp += chunk;
  }
  return 0;
}
int send_stereo_audio(complex float *buffer,int size){
  const int L = 256;
  int i,chunk;

  send_stereo_opus(buffer,size);

  // Actual number of samples read
  while(size > 0){
    struct iovec iovec[2];
    struct msghdr message;
    signed short outsamps[2*L];
    struct rtp_header rtp;

    chunk = min(size,L);

    for(i=0;i<chunk;i++){
      outsamps[2*i] = htons(CLIP(SHRT_MAX * crealf(buffer[i]))); // Scale, clip and byte swap
      outsamps[2*i+1] = htons(CLIP(SHRT_MAX * cimagf(buffer[i]))); // Scale, clip and byte swap
    }
    rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
    rtp.ssrc = htonl(PCM_ssrc);
    rtp.mpt = 10;         // Stereo linear 16-bit pcm 48 kHz
    rtp.seq = htons(PCM_seq++);
    rtp.timestamp = htonl(PCM_timestamp);
    PCM_timestamp += chunk;
    
    iovec[0].iov_base = &rtp;
    iovec[0].iov_len = sizeof(rtp);
    iovec[1].iov_base = outsamps;
    iovec[1].iov_len = 2*sizeof(outsamps[0])*chunk; // 2 bytes/sample * stereo
    
    message.msg_name = &PCM_mcast_sockaddr;
    message.msg_namelen = sizeof(PCM_mcast_sockaddr);
    message.msg_iov = &iovec[0];
    message.msg_iovlen = 2;
    message.msg_control = NULL;
    message.msg_controllen = 0;
    message.msg_flags = 0;
    sendmsg(Mcast_fd,&message,0);
    
    size -= chunk;
    buffer += chunk;
  }
  return 0;
}


int setup_audio(){
  struct group_req group_req;
  int error;
  struct sockaddr_in lsock;
  
  Mcast_fd = socket(AF_INET,SOCK_DGRAM,0);
  lsock.sin_family = AF_INET;
  lsock.sin_addr.s_addr = INADDR_ANY;
  lsock.sin_port = 0;
  if(bind(Mcast_fd,&lsock,sizeof(lsock)) == -1)
    perror("bind");

  // Make this configurable!!
  PCM_mcast_sockaddr.ss_family = AF_INET;
  ((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_port = htons(5004);
  inet_pton(AF_INET,"239.1.2.5",&((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_addr);

  group_req.gr_interface = 0;
  memcpy(&group_req.gr_group,&PCM_mcast_sockaddr,sizeof(PCM_mcast_sockaddr));
  if(setsockopt(Mcast_fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0)
    perror("setsockopt ipv4 multicast join");

    // Make this configurable!!
  OPUS_mcast_sockaddr.ss_family = AF_INET;
  ((struct sockaddr_in *)&OPUS_mcast_sockaddr)->sin_port = htons(5004);
  inet_pton(AF_INET,"239.1.2.6",&((struct sockaddr_in *)&OPUS_mcast_sockaddr)->sin_addr);

  group_req.gr_interface = 0;
  memcpy(&group_req.gr_group,&OPUS_mcast_sockaddr,sizeof(OPUS_mcast_sockaddr));
  if(setsockopt(Mcast_fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0)
    perror("setsockopt ipv4 multicast join");

  // Apparently works for both IPv4 and IPv6
  u_char loop = 1;
  if(setsockopt(Mcast_fd,IPPROTO_IP,IP_MULTICAST_LOOP,&loop,sizeof(loop)) != 0)
    perror("setsockopt multicast loop failed");

  u_char ttl = 1;
  if(setsockopt(Mcast_fd,IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof(ttl)) != 0)
    perror("setsockopt multicast ttl failed");

  time_t tt;
  time(&tt);
  PCM_ssrc = tt & 0xffffffff; // low 32 bits of clock time

  OPUS_ssrc = (tt+1) & 0xffffffff; // low 32 bits of clock time
  Opus = opus_encoder_create(48000,2,OPUS_APPLICATION_AUDIO,&error);
  opus_encoder_ctl(Opus,OPUS_SET_BITRATE(Opusbitrate));
  return 0;
}
