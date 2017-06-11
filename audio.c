// $Id: audio.c,v 1.10 2017/06/05 06:40:25 karn Exp karn $
// Multicast PCM audio
#define _GNU_SOURCE 1
#include <assert.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <complex.h>
#include <stdint.h>
#undef I
#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "rtp.h"
#include "dsp.h"
#include "audio.h"

struct audio Audio;

int PCM_fd;
long Ssrc;
struct sockaddr PCM_mcast_sockaddr;
int Timestamp = 0;
int Seq = 0;

int setup_audio(){
  PCM_fd = socket(AF_INET,SOCK_DGRAM,0);

  // Make this configurable
  PCM_mcast_sockaddr.sa_family = AF_INET;
  ((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_port = htons(54312);
  inet_pton(AF_INET,"239.1.2.5",&((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_addr);

  struct group_req group_req;
  group_req.gr_interface = 0;
  memcpy(&group_req.gr_group,&PCM_mcast_sockaddr,sizeof(PCM_mcast_sockaddr));
  if(setsockopt(PCM_fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,
		sizeof(group_req)) != 0)
    perror("setsockopt ipv4 multicast join");
  // Apparently works for both IPv4 and IPv6
  u_char loop = 1;
  if(setsockopt(PCM_fd,IPPROTO_IP,IP_MULTICAST_LOOP,&loop,sizeof(loop)) != 0)
    perror("setsockopt multicast loop failed");

  u_char ttl = 1;
  if(setsockopt(PCM_fd,IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof(ttl)) != 0)
    perror("setsockopt multicast ttl failed");

  time_t tt;
  time(&tt);
  Ssrc = tt & 0xffffffff; // low 32 bits of clock time
  return 0;
}


int send_mono_audio(float *buffer,int size){
  const int L = 512;
  struct iovec iovec[2];
  struct msghdr message;
  signed short outsamps[L];
  struct rtp_header rtp;
  int i,chunk;
  float *sp;

  // Actual number of samples read
  sp = buffer;
  while(size > 0){
    chunk = min(size,L);

    for(i=0;i<chunk;i++)
      outsamps[i] = htons(CLIP(SHRT_MAX * sp[i])); // Scale, clip and byte swap
  
    rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
    rtp.ssrc = htonl(Ssrc);
    rtp.mpt = 11;         // Mono linear 16-bit pcm 48 kHz
    rtp.seq = htons(Seq++);
    rtp.timestamp = htonl(Timestamp);
    Timestamp += chunk;
    
    iovec[0].iov_base = &rtp;
    iovec[0].iov_len = sizeof(rtp);
    iovec[1].iov_base = outsamps;
    iovec[1].iov_len = 2*chunk; // 2 bytes/sample
    
    message.msg_name = &PCM_mcast_sockaddr;
    message.msg_namelen = sizeof(PCM_mcast_sockaddr);
    message.msg_iov = &iovec[0];
    message.msg_iovlen = 2;
    message.msg_control = NULL;
    message.msg_controllen = 0;
    message.msg_flags = 0;
    sendmsg(PCM_fd,&message,0);
    
    size -= chunk;
    sp += chunk;
  }
  return 0;
}
int send_stereo_audio(complex float *buffer,int size){
  const int L = 256;
  struct iovec iovec[2];
  struct msghdr message;
  signed short outsamps[2*L];
  struct rtp_header rtp;
  int i,chunk;
  complex float *sp;

  // Actual number of samples read
  sp = buffer;
  while(size > 0){
    chunk = min(size,L);

    for(i=0;i<chunk;i++){
      outsamps[2*i] = htons(CLIP(SHRT_MAX * crealf(sp[i]))); // Scale, clip and byte swap
      outsamps[2*i+1] = htons(CLIP(SHRT_MAX * cimagf(sp[i]))); // Scale, clip and byte swap
    }
  
    rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
    rtp.ssrc = htonl(Ssrc);
    rtp.mpt = 10;         // Stereo linear 16-bit pcm 48 kHz
    rtp.seq = htons(Seq++);
    rtp.timestamp = htonl(Timestamp);
    Timestamp += chunk;
    
    iovec[0].iov_base = &rtp;
    iovec[0].iov_len = sizeof(rtp);
    iovec[1].iov_base = outsamps;
    iovec[1].iov_len = 4*chunk; // 2 bytes/sample
    
    message.msg_name = &PCM_mcast_sockaddr;
    message.msg_namelen = sizeof(PCM_mcast_sockaddr);
    message.msg_iov = &iovec[0];
    message.msg_iovlen = 2;
    message.msg_control = NULL;
    message.msg_controllen = 0;
    message.msg_flags = 0;
    sendmsg(PCM_fd,&message,0);
    
    size -= chunk;
    sp += chunk;
  }
  return 0;
}
