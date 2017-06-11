// $Id$
// Listen to multicast, send PCM audio to Linux ALSA driver
#define _GNU_SOURCE 1
#include <assert.h>
#include <stdio.h>
#include <alsa/asoundlib.h>
#include <limits.h>
#include <string.h>
#include <opus/opus.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "rtp.h"
#include "dsp.h"

struct {
  char *name;
  snd_pcm_t *handle;
  int samprate;
  int underrun;
  int overrun;
} Audio;

int audio_out_done(){
  snd_pcm_drop(Audio.handle);
  snd_pcm_close(Audio.handle);
  Audio.handle = NULL;
  return 0;
}

// Set up or change ALSA for demodulated sound output
int audio_change_parms(unsigned samprate,int channels,int L){
  snd_pcm_uframes_t buffer_size;
  unsigned actual_rate;
  snd_pcm_hw_params_t *hw_params;
  snd_pcm_sw_params_t *sw_params;

#if 0
  // This is a nice informative message, but it gets called every time we change modes
  fprintf(stderr,"audio_change_parms(%s,rate=%u,blksize=%d,chans=%d)\n",Audio.name,samprate,L,channels);
#endif
  if(Audio.handle != NULL){
    snd_pcm_drop(Audio.handle);
    snd_pcm_close(Audio.handle);
    Audio.handle = NULL;
  }

  if(snd_pcm_open(&Audio.handle,Audio.name,SND_PCM_STREAM_PLAYBACK,0) < 0){
    fprintf(stderr,"Error opening D/A %s\n",Audio.name);
    return -1;
  }

  snd_pcm_hw_params_alloca(&hw_params);
  if(snd_pcm_hw_params_any(Audio.handle,hw_params) < 0){
    fprintf(stderr,"Can not configure D/A %s\n",Audio.name);
    return -1;
  }
  if(snd_pcm_hw_params_set_access(Audio.handle,hw_params,SND_PCM_ACCESS_RW_INTERLEAVED) < 0){
    fprintf(stderr,"Error setting D/A access on %s\n",Audio.name);
    return -1;
  }
  if(snd_pcm_hw_params_set_format(Audio.handle,hw_params,SND_PCM_FORMAT_S16_LE)<0){
    fprintf(stderr,"Error setting D/A format on %s\n",Audio.name);
    return -1;
  }
  actual_rate = samprate;
  if(snd_pcm_hw_params_set_rate_near(Audio.handle,hw_params,&actual_rate,0)<0){
    fprintf(stderr,"Error setting D/A sample rate on %s\n",Audio.name);
    return -1;
  }
  if(actual_rate != samprate)
    fprintf(stderr,"Warning: actual D/A sample rate is %u\n",actual_rate);

  Audio.samprate = actual_rate;
  if(snd_pcm_hw_params_set_channels(Audio.handle,hw_params,channels)<0){ // stereo
    fprintf(stderr,"Error setting D/A channels on %s\n",Audio.name);
    return -1;
  }
  // We will generally write L-sample blocks at a time
  snd_pcm_uframes_t LL = L;
  if(snd_pcm_hw_params_set_period_size_near(Audio.handle,hw_params,&LL,0)<0){
    fprintf(stderr,"Error setting D/A periods on %s\n",Audio.name);
    return -1;
  }
  if(LL != L){
    fprintf(stderr,"D/A periods: requested %d, got %d\n",(int)L,(int)LL);
  }
  buffer_size = 8*LL;
  if(snd_pcm_hw_params_set_buffer_size_near(Audio.handle,hw_params,&buffer_size)<0){
    fprintf(stderr,"Error setting D/A buffersize on %s\n",Audio.name);
    return -1;
  }
  if(buffer_size != 8*LL){
    fprintf(stderr,"D/A buffer: requested %d, got %d\n",(int)(8*LL),(int)buffer_size);
  }

  if(snd_pcm_hw_params(Audio.handle,hw_params)<0){
    fprintf(stderr,"Error setting D/A HW params on %s\n",Audio.name);
    return -1;
  }
  snd_pcm_sw_params_alloca(&sw_params);
  if(snd_pcm_sw_params_current(Audio.handle,sw_params) < 0){
    fprintf(stderr,"Can not configure driver for %s\n",Audio.name);
    return -1;
  }
  if(snd_pcm_sw_params_set_start_threshold(Audio.handle,sw_params,2*LL) < 0){
    fprintf(stderr,"Can not configure start threshold for %s\n",Audio.name);
    return -1;
  }
#if 0
  if(snd_pcm_sw_params_set_stop_threshold(Audio.handle,sw_params,8*LL) < 0){
    fprintf(stderr,"Can not configure stop threshold for %s\n",Audio.name);
    return -1;
  }
#endif
  if(snd_pcm_sw_params(Audio.handle,sw_params) < 0){
    fprintf(stderr,"Can not set sw params %s\n",Audio.name);
  }
  return 0;
}

struct sockaddr PCM_mcast_sockaddr,PCM_sender;
int PCM_fd;

int main(int argc,char *argv[]){
  const int L=2048;
  int chunk;
  struct rtp_header rtp;
  uint16_t data[2*L];
  struct sockaddr sender;

  Audio.name = "default";
  Audio.samprate = 48000;
  
  audio_change_parms(Audio.samprate,2,L);

  // Make this configurable!
  PCM_fd = socket(AF_INET,SOCK_DGRAM,0);
  PCM_mcast_sockaddr.sa_family = AF_INET;
  ((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_port = htons(54312);
  inet_pton(AF_INET,"239.1.2.5",&((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_addr);

  struct group_req group_req;
  group_req.gr_interface = 0;
  memcpy(&group_req.gr_group,&PCM_mcast_sockaddr,sizeof(PCM_mcast_sockaddr));
  if(setsockopt(PCM_fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0)
    perror("setsockopt ipv4 multicast join");
  // Apparently works for both IPv4 and IPv6
  u_char loop = 1;
  if(setsockopt(PCM_fd,IPPROTO_IP,IP_MULTICAST_LOOP,&loop,sizeof(loop)) != 0)
    perror("setsockopt multicast loop failed");

  u_char ttl = 1;
  if(setsockopt(PCM_fd,IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof(ttl)) != 0)
    perror("setsockopt multicast ttl failed");

  if(bind(PCM_fd,&PCM_mcast_sockaddr,sizeof(PCM_mcast_sockaddr)) == -1){
    perror("bind");
    exit(1);
  }
  u_char reuse = 1;
  if(setsockopt(PCM_fd,IPPROTO_IP,SO_REUSEADDR,&reuse,sizeof(reuse)) != 0)
    perror("ipv4 so_reuseaddr failed");

  char dest[INET6_ADDRSTRLEN];
  int dport = -1;
  if(PCM_mcast_sockaddr.sa_family == AF_INET){
    inet_ntop(AF_INET,&((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_addr,dest,sizeof(dest));      
    dport = ntohs(((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_port);
  } else if(PCM_mcast_sockaddr.sa_family == AF_INET6){
    inet_ntop(AF_INET6,&((struct sockaddr_in6 *)&PCM_mcast_sockaddr)->sin6_addr,dest,sizeof(dest));
    dport = ntohs(((struct sockaddr_in6 *)&PCM_mcast_sockaddr)->sin6_port);
  }

  fprintf(stderr,"Listening on %s:%d\n",dest,dport);

  struct iovec iovec[2];
  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = data;
  iovec[1].iov_len = sizeof(data);

  struct msghdr message;
  message.msg_name = &sender;
  message.msg_namelen = sizeof(sender);
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 2;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  while(1){
    ssize_t size;
    int i,r;
    uint16_t outsamps[8192],*sp;

    size = recvmsg(PCM_fd,&message,0);

    if(size < sizeof(rtp))
      continue;


    size -= sizeof(rtp); // Bytes in data
    size /= 2;           // 16-bit words in data; equal to # mono samples or 2x # stereo samples
    switch(rtp.mpt){
    case 10: // Stereo
      for(i=0;i<size;i++)
	outsamps[i] = ntohs(data[i]);
      size /= 2;           // # stereo samples
      break;
    case 11: // Mono; convert to stereo
      for(i=0;i<size;i++)
	outsamps[2*i] = outsamps[2*i+1] = ntohs(data[i]);
      break;
    }

    sp = outsamps;
    while(size > 0){
      snd_pcm_state_t state;
      state = snd_pcm_state(Audio.handle);
      // Underruns can deliberately happen when the demodulator thread simply
      // stops sending data, e.g., when a FM squelch is closed
      if(state != SND_PCM_STATE_RUNNING && state != SND_PCM_STATE_PREPARED){
	Audio.underrun++;
	snd_pcm_prepare(Audio.handle);
      }
      chunk = snd_pcm_avail(Audio.handle); // stereo samples available in sound buffer
      if(chunk == 0){
	Audio.overrun++;
	usleep(L * 1e6 / Audio.samprate); // Wait one buffer time for some room
	continue;
      }
      // Size of the buffer, or the max available in the device, whichever is less
      chunk = min(chunk,size);
      if((r = snd_pcm_writei(Audio.handle,sp,chunk)) != chunk){
#if 0
	fprintf(stderr,"audio write fail %s %d %s\n",snd_strerror(r),r,strerror(-r));
#endif
      }
      size -= r; // stereo samples left (2 16 bit words, 4 bytes)
      sp += 2*r; // Each stereo sample is 2 16-bit words
    }
    // Show sending address & source port, if it has changed
    if(memcmp(&PCM_sender,&sender,sizeof(sender)) != 0){
      memcpy(&PCM_sender,&sender,sizeof(sender));
      char src[INET6_ADDRSTRLEN];
      int dport = -1;
      if(PCM_sender.sa_family == AF_INET){
	inet_ntop(AF_INET,&((struct sockaddr_in *)&PCM_sender)->sin_addr,src,sizeof(src));      
	dport = ntohs(((struct sockaddr_in *)&PCM_sender)->sin_port);
      } else if(PCM_mcast_sockaddr.sa_family == AF_INET6){
	inet_ntop(AF_INET6,&((struct sockaddr_in6 *)&PCM_sender)->sin6_addr,src,sizeof(src));
	dport = ntohs(((struct sockaddr_in6 *)&PCM_sender)->sin6_port);
      }
      fprintf(stderr,"Receiving from %s:%d\n",src,dport);
    }
  }
  audio_out_done();
  close(PCM_fd);
  exit(0);
}
