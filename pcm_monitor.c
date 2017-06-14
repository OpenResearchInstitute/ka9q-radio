// $Id: pcm_monitor.c,v 1.7 2017/06/14 03:09:12 karn Exp karn $
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
  int etime; // Next expected timestamp
  time_t lastused;
  struct sockaddr_storage pcm_sender;
  OpusDecoder *opus;
  char *name;
  snd_pcm_t *handle;
  unsigned int samprate;
  int underrun;
  int overrun;
} Audio[NSESSIONS];


void close_audio(struct audio *ap){
  if(ap->opus != NULL){
    opus_decoder_destroy(ap->opus);
    ap->opus = NULL;
  }
  if(ap->handle){
    snd_pcm_drop(ap->handle);
    snd_pcm_close(ap->handle);
    ap->handle = NULL;
  }
}

void closedown(){
  int i;
  for(i=0;i<NSESSIONS;i++)
    close_audio(&Audio[i]);

  if(Mcast_fd != -1)
    close(Mcast_fd);
  exit(0);
}



// Set up or change ALSA for demodulated sound output
int audio_init(struct audio *ap,unsigned samprate,int channels,int L){
  snd_pcm_uframes_t buffer_size;
  snd_pcm_hw_params_t *hw_params;
  snd_pcm_sw_params_t *sw_params;

#if 0
  // This is a nice informative message, but it gets called every time we change modes
  fprintf(stderr,"audio_init(%s,rate=%u,blksize=%d,chans=%d)\n",ap->name,samprate,L,channels);
#endif

  if(ap->opus != NULL)
    opus_decoder_destroy(ap->opus);

  int error;
  ap->opus = opus_decoder_create(samprate,2,&error);
  if(ap->opus == NULL)
    fprintf(stderr,"Opus decoder error %d\n",error);

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
  ap->samprate = samprate;
  if(snd_pcm_hw_params_set_rate_near(ap->handle,hw_params,&ap->samprate,0)<0){
    fprintf(stderr,"Error setting D/A sample rate on %s\n",ap->name);
    return -1;
  }
  if(ap->samprate != samprate)
    fprintf(stderr,"Warning: actual D/A sample rate is %u\n",ap->samprate);

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
  ap->underrun = 0;
  ap->overrun = 0;
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
      // What errors could occur here? Discard buffer to catch up
#if 1
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
  char *mcast_address_string = "239.1.2.6"; // Opus broadcast
  int dport = 5004;

  int c;
  while((c = getopt(argc,argv,"S:I:P:")) != EOF){
    switch(c){
    case 'S':
      Audioname = optarg;
      break;
    case 'I':
      mcast_address_string = optarg;
      break;
    case 'P':
      dport = atoi(optarg);
      break;
    default:
      fprintf(stderr,"Usage: %s [-S audioname] [-I mcast_address] [-P dport]\n",argv[0]);
      fprintf(stderr,"Defaults: %s -S %s -I %s -P %d\n",argv[0],Audioname,mcast_address_string,dport);
      exit(1);
    }
  }
  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);

  // Set up socket, bind to port
  // Add full IPv6 support!
  Mcast_fd = socket(AF_INET,SOCK_DGRAM,0);
  inet_pton(AF_INET,mcast_address_string,&((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_addr);
  PCM_mcast_sockaddr.ss_family = AF_INET;
  ((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_port = htons(dport);

  if(IN_MULTICAST(ntohl((((struct sockaddr_in *)&PCM_mcast_sockaddr)->sin_addr.s_addr)))){
    // Join only if it's actually a multicast address
    // We could join 0.0.0.0 or our own address to receive unicasts
    struct group_req group_req;
    group_req.gr_interface = 0;
    memcpy(&group_req.gr_group,&PCM_mcast_sockaddr,sizeof(PCM_mcast_sockaddr));
    if(setsockopt(Mcast_fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0)
      perror("setsockopt ipv4 multicast join");
  }
  // Let other processes also bind to this port
  int reuse = 1;
  if(setsockopt(Mcast_fd,SOL_SOCKET,SO_REUSEPORT,&reuse,sizeof(reuse)) != 0)
    perror("so_reuseaddr");

  if(bind(Mcast_fd,(struct sockaddr_in *)&PCM_mcast_sockaddr,sizeof(PCM_mcast_sockaddr)) == -1){
    perror("bind");
    exit(1);
  }

  fprintf(stderr,"Listening on %s:%d\n",mcast_address_string,dport);

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

    int drop;

    if((size = recvmsg(Mcast_fd,&message,0)) < sizeof(rtp))
      continue; // Too small to be valid RTP

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
	  // Initialize entry
	  memset(&ap->pcm_sender,0,sizeof(ap->pcm_sender));
	  ap->name = Audioname;
	  audio_init(ap,Samprate,2,L);
	  ap->ssrc = rtp.ssrc;
	  ap->eseq = rtp.seq;
	  ap->etime = rtp.timestamp;
	  break;
	}
      }
    }
    if(ap == NULL){
      char src[INET6_ADDRSTRLEN];
      int sport = -1;
      if(sender.ss_family == AF_INET){
	inet_ntop(AF_INET,&((struct sockaddr_in *)&sender)->sin_addr,src,sizeof(src));      
	sport = ntohs(((struct sockaddr_in *)&sender)->sin_port);
      } else if(sender.ss_family == AF_INET6){
	inet_ntop(AF_INET6,&((struct sockaddr_in6 *)&sender)->sin6_addr,src,sizeof(src));
	sport = ntohs(((struct sockaddr_in6 *)&sender)->sin6_port);
      } else {
	strcpy(src,"unknown");
      }
      fprintf(stderr,"No slots available for ssrc 0x%x from %s:%d\n",(unsigned int)rtp.ssrc,src,sport);
      continue; // Drop packet
    }
    ap->lastused = time(NULL);
    if(rtp.seq != ap->eseq){
      fprintf(stderr,"expected %d got %d\n",ap->eseq,rtp.seq);
      if((int16_t)(rtp.seq - ap->eseq) < 0){
	ap->eseq = (rtp.seq + 1) & 0xffff;
	continue;	// Drop probable duplicate
      } else
	drop = rtp.seq - ap->eseq;
    } else
      drop = 0;
    ap->eseq = (rtp.seq + 1) & 0xffff;
    // How do we compute next expected timestamp?

    // Show sending address & source port, if it has changed
    if(memcmp(&ap->pcm_sender,&sender,sizeof(sender)) != 0){
      memcpy(&ap->pcm_sender,&sender,sizeof(sender));
      char src[INET6_ADDRSTRLEN];
      int sport = -1;
      if(sender.ss_family == AF_INET){
	inet_ntop(AF_INET,&((struct sockaddr_in *)&sender)->sin_addr,src,sizeof(src));      
	sport = ntohs(((struct sockaddr_in *)&sender)->sin_port);
      } else if(PCM_mcast_sockaddr.ss_family == AF_INET6){
	inet_ntop(AF_INET6,&((struct sockaddr_in6 *)&sender)->sin6_addr,src,sizeof(src));
	sport = ntohs(((struct sockaddr_in6 *)&sender)->sin6_port);
      }
      fprintf(stderr,"Session %ld receiving ssrc %x from %s:%d\n",(long int)(ap - Audio),ap->ssrc,src,sport);
    }

    size -= sizeof(rtp); // Bytes in data
    int16_t outsamps[Bufsize];
    switch(rtp.mpt){
    case 10: // Stereo
      size /= 2;           // # 16-bit word samples
      for(i=0;i<size;i++)
	outsamps[i] = ntohs(data[i]); // RTP profile specifies big-endian samples
      size /= 2;           // # 32-bit stereo samples
      break;
    case 11: // Mono; send to both stereo channels
      size /= 2;
      for(i=0;i<size;i++)
	outsamps[2*i] = outsamps[2*i+1] = ntohs(data[i]);
      break;
    case 20: // Opus codec decode - arbitrary choice
      if(drop != 0){
	// packet dropped; conceal
	// Should give # samples in missing packet, but how?
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
