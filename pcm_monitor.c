// $Id: pcm_monitor.c,v 1.3 2017/06/13 02:52:07 karn Exp karn $
// Listen to multicast, send PCM audio to Linux ALSA driver
#define _GNU_SOURCE 1
#include <assert.h>
#include <stdio.h>
#include <signal.h>
#include <alsa/asoundlib.h>
#include <limits.h>
#include <string.h>
#include <opus/opus.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "rtp.h"
#include "dsp.h"

// Maximum samples/words per packet
// Bigger than Ethernet MTU because of network device fragmentation/reassembly
const int Bufsize = 8192;
struct sockaddr_storage PCM_mcast_sockaddr;
int Mcast_fd;
const int L=2048;
const int Samprate = 48000;
char *Audioname = "default";


#define NSESSIONS 20

struct audio {
  uint32_t ssrc;
  int eseq; // Next expected sequence number
  time_t lastused;
  struct sockaddr_storage pcm_sender;
  OpusDecoder *opus;
  char *name;
  snd_pcm_t *handle;
  int samprate;
  int underrun;
  int overrun;
} Audio[NSESSIONS];



void close_audio(struct audio *ap){
  if(ap->opus != NULL)
    opus_decoder_destroy(ap->opus);
  ap->opus = NULL;
  ap->eseq = -1;
  if(ap->handle){
    snd_pcm_drop(ap->handle);
    snd_pcm_close(ap->handle);
    ap->handle = NULL;
  }
  memset(&ap->pcm_sender,0,sizeof(ap->pcm_sender));
}

void closedown(){
  int i;
  for(i=0;i<NSESSIONS;i++){
    close_audio(&Audio[i]);
  }
  exit(0);
}



// Set up or change ALSA for demodulated sound output
int audio_change_parms(struct audio *ap,unsigned samprate,int channels,int L){
  snd_pcm_uframes_t buffer_size;
  unsigned actual_rate;
  snd_pcm_hw_params_t *hw_params;
  snd_pcm_sw_params_t *sw_params;

#if 0
  // This is a nice informative message, but it gets called every time we change modes
  fprintf(stderr,"audio_change_parms(%s,rate=%u,blksize=%d,chans=%d)\n",ap->name,samprate,L,channels);
#endif
  int error;
  if(ap->opus != NULL)
    opus_decoder_destroy(ap->opus);

  ap->opus = opus_decoder_create(samprate,2,&error);
  if(ap->opus == NULL){
    fprintf(stderr,"Opus decoder error %d\n",error);
  }

  if(ap->handle != NULL){
    snd_pcm_drop(ap->handle);
    snd_pcm_close(ap->handle);
    ap->handle = NULL;
  }

  if(snd_pcm_open(&ap->handle,ap->name,SND_PCM_STREAM_PLAYBACK,0) < 0){
    fprintf(stderr,"Error opening D/A %s\n",ap->name);
    return -1;
  }

  snd_pcm_hw_params_alloca(&hw_params);
  if(snd_pcm_hw_params_any(ap->handle,hw_params) < 0){
    fprintf(stderr,"Can not configure D/A %s\n",ap->name);
    return -1;
  }
  if(snd_pcm_hw_params_set_access(ap->handle,hw_params,SND_PCM_ACCESS_RW_INTERLEAVED) < 0){
    fprintf(stderr,"Error setting D/A access on %s\n",ap->name);
    return -1;
  }
  if(snd_pcm_hw_params_set_format(ap->handle,hw_params,SND_PCM_FORMAT_S16_LE)<0){
    fprintf(stderr,"Error setting D/A format on %s\n",ap->name);
    return -1;
  }
  actual_rate = samprate;
  if(snd_pcm_hw_params_set_rate_near(ap->handle,hw_params,&actual_rate,0)<0){
    fprintf(stderr,"Error setting D/A sample rate on %s\n",ap->name);
    return -1;
  }
  if(actual_rate != samprate)
    fprintf(stderr,"Warning: actual D/A sample rate is %u\n",actual_rate);

  ap->samprate = actual_rate;
  if(snd_pcm_hw_params_set_channels(ap->handle,hw_params,channels)<0){ // stereo
    fprintf(stderr,"Error setting D/A channels on %s\n",ap->name);
    return -1;
  }
  // We will generally write L-sample blocks at a time
  snd_pcm_uframes_t LL = L;
  if(snd_pcm_hw_params_set_period_size_near(ap->handle,hw_params,&LL,0)<0){
    fprintf(stderr,"Error setting D/A periods on %s\n",ap->name);
    return -1;
  }
  if(LL != L){
    fprintf(stderr,"D/A periods: requested %d, got %d\n",(int)L,(int)LL);
  }
  buffer_size = 8*LL;
  if(snd_pcm_hw_params_set_buffer_size_near(ap->handle,hw_params,&buffer_size)<0){
    fprintf(stderr,"Error setting D/A buffersize on %s\n",ap->name);
    return -1;
  }
  if(buffer_size != 8*LL){
    fprintf(stderr,"D/A buffer: requested %d, got %d\n",(int)(8*LL),(int)buffer_size);
  }

  if(snd_pcm_hw_params(ap->handle,hw_params)<0){
    fprintf(stderr,"Error setting D/A HW params on %s\n",ap->name);
    return -1;
  }
  snd_pcm_sw_params_alloca(&sw_params);
  if(snd_pcm_sw_params_current(ap->handle,sw_params) < 0){
    fprintf(stderr,"Can not configure driver for %s\n",ap->name);
    return -1;
  }
  if(snd_pcm_sw_params_set_start_threshold(ap->handle,sw_params,2*LL) < 0){
    fprintf(stderr,"Can not configure start threshold for %s\n",ap->name);
    return -1;
  }
#if 0
  if(snd_pcm_sw_params_set_stop_threshold(ap->handle,sw_params,8*LL) < 0){
    fprintf(stderr,"Can not configure stop threshold for %s\n",ap->name);
    return -1;
  }
#endif
  if(snd_pcm_sw_params(ap->handle,sw_params) < 0){
    fprintf(stderr,"Can not set sw params %s\n",ap->name);
  }
  return 0;
}

// play buffer of 'size' samples, each 16 bit stereo (4*size bytes)
int play_stereo_pcm(struct audio *ap,int16_t *outsamps,int size){
    while(size > 0){
      int chunk,r;

      snd_pcm_state_t state;
      state = snd_pcm_state(ap->handle);
      // Underruns can deliberately happen when the demodulator thread simply
      // stops sending data, e.g., when a FM squelch is closed
      if(state != SND_PCM_STATE_RUNNING && state != SND_PCM_STATE_PREPARED){
	ap->underrun++;
	snd_pcm_prepare(ap->handle);
      }
      chunk = snd_pcm_avail(ap->handle); // stereo samples available in sound buffer
      if(chunk == 0){
	ap->overrun++;
	usleep(L * 1000000 / ap->samprate); // Wait one buffer time for some room
	continue;
      }
      // Size of the buffer, or the max available in the device, whichever is less
      chunk = min(chunk,size);
      if((r = snd_pcm_writei(ap->handle,outsamps,chunk)) != chunk){
#if 0
	fprintf(stderr,"audio write fail %s %d %s\n",snd_strerror(r),r,strerror(-r));
#endif
      }
      size -= r; // stereo samples left (2 16 bit words, 4 bytes)
      outsamps += 2*r; // Each stereo sample is 2 16-bit words
    }
    return 0;
}


int main(int argc,char *argv[]){
  const int L=2048;

  int c;
  char *mcast_address_string = "239.1.2.6"; // Opus broadcast
  int mcast_port = 5004;

  while((c = getopt(argc,argv,"S:R:P:")) != EOF){
    switch(c){
    case 'S':
      Audioname = optarg;
      break;
    case 'R':
      mcast_address_string = optarg;
      break;
    case 'P':
      mcast_port = atoi(optarg);
      break;
    }
  }
  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);

  // Set up and join multicast group
  Mcast_fd = socket(AF_INET,SOCK_DGRAM,0);
  PCM_mcast_sockaddr.ss_family = AF_INET;
  ((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_port = htons(mcast_port);
  inet_pton(AF_INET,mcast_address_string,&((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_addr);

  struct group_req group_req;
  group_req.gr_interface = 0;
  memcpy(&group_req.gr_group,&PCM_mcast_sockaddr,sizeof(PCM_mcast_sockaddr));
  if(setsockopt(Mcast_fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0)
    perror("setsockopt ipv4 multicast join");
#if 0 // We're only listening
  // Apparently works for both IPv4 and IPv6
  u_char loop = 1;
  if(setsockopt(Mcast_fd,IPPROTO_IP,IP_MULTICAST_LOOP,&loop,sizeof(loop)) != 0)
    perror("setsockopt multicast loop failed");

  u_char ttl = 1;
  if(setsockopt(Mcast_fd,IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof(ttl)) != 0)
    perror("setsockopt multicast ttl failed");
#endif

  char dest[INET6_ADDRSTRLEN];
  int dport = -1;

  if(PCM_mcast_sockaddr.ss_family == AF_INET){
    if(bind(Mcast_fd,(struct sockaddr_in *)&PCM_mcast_sockaddr,sizeof(PCM_mcast_sockaddr)) == -1){
      perror("bind");
      exit(1);
    }
    inet_ntop(AF_INET,&((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_addr,dest,sizeof(dest));      
    dport = ntohs(((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_port);
  } else if(PCM_mcast_sockaddr.ss_family == AF_INET6){
    if(bind(Mcast_fd,(struct sockaddr_in6 *)&PCM_mcast_sockaddr,sizeof(PCM_mcast_sockaddr)) == -1){
      perror("bind");
      exit(1);
    }
    inet_ntop(AF_INET6,&((struct sockaddr_in6 *)&PCM_mcast_sockaddr)->sin6_addr,dest,sizeof(dest));
    dport = ntohs(((struct sockaddr_in6 *)&PCM_mcast_sockaddr)->sin6_port);
  } else {
    fprintf(stderr,"Unknown multicast address family %d\n",PCM_mcast_sockaddr.ss_family);
    exit(1);
  }
  // Let other processes also bind to this port
  u_char reuse = 1;
  if(setsockopt(Mcast_fd,IPPROTO_IP,SO_REUSEADDR,&reuse,sizeof(reuse)) != 0)
    perror("so_reuseaddr");

  fprintf(stderr,"Listening on %s:%d\n",dest,dport);

  struct iovec iovec[2];
  struct rtp_header rtp;
  int16_t data[2*L];
  
  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = data;
  iovec[1].iov_len = sizeof(data);

  struct msghdr message;
  struct sockaddr_storage sender;
  message.msg_name = &sender;
  message.msg_namelen = sizeof(sender);
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 2;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  while(1){
    ssize_t size;
    int16_t outsamps[Bufsize];
    int drop;

    if((size = recvmsg(Mcast_fd,&message,0)) < sizeof(rtp))
      continue;

    // To host order
    rtp.ssrc = ntohl(rtp.ssrc);
    rtp.seq = ntohs(rtp.seq);
    rtp.timestamp = ntohl(rtp.timestamp);

    // Look up session
    struct audio *ap = NULL;
    int i;

    for(i=0;i<NSESSIONS;i++){
      if(Audio[i].ssrc == rtp.ssrc){
	ap = &Audio[i];
	break;
      }
    }
    if(ap == NULL){ // Not found; look for empty slot
      time_t tt = time(NULL);
      for(i=0;i<NSESSIONS;i++){
	if(Audio[i].handle == NULL || Audio[i].lastused + 30 < tt){
	  ap = &Audio[i];
	  if(ap->handle != NULL){
	    fprintf(stderr,"Closing old session %d\n",i);
	    close_audio(ap);
	  }
	  break;
	}
      }
    }
    if(ap == NULL){
      char src[INET6_ADDRSTRLEN];
      int dport = -1;
      if(sender.ss_family == AF_INET){
	inet_ntop(AF_INET,&((struct sockaddr_in *)&sender)->sin_addr,src,sizeof(src));      
	dport = ntohs(((struct sockaddr_in *)&sender)->sin_port);
      } else if(sender.ss_family == AF_INET6){
	inet_ntop(AF_INET6,&((struct sockaddr_in6 *)&sender)->sin6_addr,src,sizeof(src));
	dport = ntohs(((struct sockaddr_in6 *)&sender)->sin6_port);
      } else {
	strcpy(src,"unknown");
      }
      fprintf(stderr,"No slots available for ssrc 0x%x from %s:%d\n",(unsigned int)rtp.ssrc,src,dport);
      continue; // Drop packet
    } else if(ap->handle == NULL){
      // Create new entry
      ap->name = Audioname;
      audio_change_parms(ap,Samprate,2,L);
      ap->ssrc = rtp.ssrc;
      ap->eseq = -1;
    }
    ap->lastused = time(NULL);
    if(ap->eseq != -1 && rtp.seq != ap->eseq){
      fprintf(stderr,"expected %d got %d\n",ap->eseq,rtp.seq);
      if((int16_t)(rtp.seq - ap->eseq) < 0){
	ap->eseq = (rtp.seq + 1) & 0xffff;
	continue;	// Drop probable duplicate
      } else
	drop = rtp.seq - ap->eseq;
    } else
      drop = 0;
    ap->eseq = (rtp.seq + 1) & 0xffff;

    // Show sending address & source port, if it has changed
    if(memcmp(&ap->pcm_sender,&sender,sizeof(sender)) != 0){
      memcpy(&ap->pcm_sender,&sender,sizeof(sender));
      char src[INET6_ADDRSTRLEN];
      int dport = -1;
      if(sender.ss_family == AF_INET){
	inet_ntop(AF_INET,&((struct sockaddr_in *)&sender)->sin_addr,src,sizeof(src));      
	dport = ntohs(((struct sockaddr_in *)&sender)->sin_port);
      } else if(PCM_mcast_sockaddr.ss_family == AF_INET6){
	inet_ntop(AF_INET6,&((struct sockaddr_in6 *)&sender)->sin6_addr,src,sizeof(src));
	dport = ntohs(((struct sockaddr_in6 *)&sender)->sin6_port);
      }
      fprintf(stderr,"Session %ld receiving ssrc %lx from %s:%d\n",ap - Audio,(unsigned long)ap->ssrc,src,dport);
    }

    size -= sizeof(rtp); // Bytes in data
    switch(rtp.mpt){
    case 10: // Stereo
      size /= 2;           // # stereo samples
      for(i=0;i<size;i++)
	outsamps[i] = ntohs(data[i]);
      break;
    case 11: // Mono; convert to stereo
      for(i=0;i<size/2;i++)
	outsamps[2*i] = outsamps[2*i+1] = ntohs(data[i]);
      break;
    case 20: // Opus codec decode - arbitrary choice
      if(drop != 0){
	// packet dropped; conceal
	size = opus_decode(ap->opus,NULL,0,(opus_int16 *)outsamps,sizeof(outsamps),0);
	play_stereo_pcm(ap,outsamps,size);
      }
      size = opus_decode(ap->opus,(unsigned char *)data,size,(opus_int16 *)outsamps,sizeof(outsamps),0);
      break;
    default:
      continue; // ignore
    }
    play_stereo_pcm(ap,outsamps,size);
  }
  exit(0);
}
