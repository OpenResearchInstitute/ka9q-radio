// $Id: pcm_monitor.c,v 1.13 2017/06/18 19:33:06 karn Exp karn $
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
struct sockaddr_in PCM_mcast_sockaddr;
int Input_fd;
const int L=512;
const int Samprate = 48000;
char *Audioname = "default";

int Verbose;


#define NSESSIONS 20

struct audio {
  struct audio *prev; // Linked list pointers
  struct audio *next; 
  uint32_t ssrc;
  int eseq; // Next expected sequence number
  int etime; // Next expected timestamp
  time_t lastused;
  struct sockaddr_in sender;
  OpusDecoder *opus;
  char *name;
  snd_pcm_t *handle;
  unsigned int samprate;
  int underrun;
  int overrun;
};

struct audio *Audio[256]; // Hash chains

void close_audio(struct audio *ap){
  if(ap == NULL)
    return;
  
  if(ap->opus != NULL){
    opus_decoder_destroy(ap->opus);
    ap->opus = NULL;
  }
  if(ap->handle){
    snd_pcm_drop(ap->handle);
    snd_pcm_close(ap->handle);
    ap->handle = NULL;
  }
  free(ap);
}

void closedown(){

  if(Input_fd != -1)
    close(Input_fd);
  exit(0);
}



// Set up or change ALSA for demodulated sound output
int audio_init(struct audio *ap,unsigned samprate,int channels,int L){
  snd_pcm_uframes_t buffer_size;
  snd_pcm_hw_params_t *hw_params;
  snd_pcm_sw_params_t *sw_params;

  if(Verbose)
    fprintf(stderr,"audio_init(%s,rate=%u,blksize=%d,chans=%d)\n",ap->name,samprate,L,channels);

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
  if(LL != L)
    fprintf(stderr,"D/A periods: requested %d, got %d\n",(int)L,(int)LL);

  buffer_size = 32*LL;
  if(snd_pcm_hw_params_set_buffer_size_near(ap->handle,hw_params,&buffer_size)<0){
    fprintf(stderr,"Error setting D/A buffersize on %s\n",ap->name);
    return -1;
  }
  if(buffer_size != 32*LL)
    fprintf(stderr,"D/A buffer: requested %d, got %d\n",(int)(32*LL),(int)buffer_size);

  if(snd_pcm_hw_params(ap->handle,hw_params)<0){
    fprintf(stderr,"Error setting D/A HW params on %s\n",ap->name);
    return -1;
  }
  snd_pcm_sw_params_alloca(&sw_params);
  if(snd_pcm_sw_params_current(ap->handle,sw_params) < 0){
    fprintf(stderr,"Can not configure driver for %s\n",ap->name);
    return -1;
  }
  // !!!
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
  if(snd_pcm_sw_params(ap->handle,sw_params) < 0)
    fprintf(stderr,"Can not set sw params %s\n",ap->name);

  ap->underrun = 0;
  ap->overrun = 0;
  return 0;
}

// play buffer of 'size' samples, each 16 bit stereo (4*size bytes)
int play_stereo_pcm(struct audio *sp,int16_t *outsamps,int size){

  static int16_t silence[960]; // Play this when we underrun; 10 ms @ 48 kHz stereo

  snd_pcm_state_t state = snd_pcm_state(sp->handle);
  // Underruns can deliberately happen when the demodulator thread simply
  // stops sending data, e.g., when a FM squelch is closed
  if(state != SND_PCM_STATE_RUNNING && state != SND_PCM_STATE_PREPARED){
    snd_pcm_prepare(sp->handle);
    if(Verbose)
      fprintf(stderr,"session %p state %d\n",sp,state);

    if(state == SND_PCM_STATE_XRUN){
      sp->underrun++;
    }
  }

  while(size > 0){
    int r;
    
    snd_pcm_sframes_t delay = 0;
    snd_pcm_sframes_t chunk = 0;
    if((r = snd_pcm_avail_delay(sp->handle,&chunk,&delay)) != 0){
      snd_pcm_prepare(sp->handle);
      if(Verbose > 1)
	fprintf(stderr,"snd_pcm_avail_delay %s %d %s\n",snd_strerror(r),r,strerror(-r));
      usleep(1000);
      continue;
    }
    if(Verbose > 1)
      fprintf(stderr,"chunk %d delay %f ms\n",(int)chunk,1000.*delay/sp->samprate);

    if(1000.*delay/sp->samprate < 10){
      // Less than 10 ms in buffer, we risk underrrun. Inject silence
      if(Verbose)
	fprintf(stderr,"%lx injecting silence\n",(long unsigned int)sp->ssrc);
      snd_pcm_writei(sp->handle,silence,sizeof(silence) / (2 * sizeof(int16_t)));
    }

    if(chunk == 0){
      sp->overrun++;
      fprintf(stderr,"session %p overrun\n",sp);
      return 0; // Drop audio to let it catch up
    }
    // Size of the buffer, or the max available in the device, whichever is less
    chunk = min(chunk,size);
    if((r = snd_pcm_writei(sp->handle,outsamps,chunk)) != chunk){
      // What errors could occur here? probably just underruns
      if(Verbose)
	fprintf(stderr,"audio write fail %s %d %s\n",snd_strerror(r),r,strerror(-r));
      snd_pcm_prepare(sp->handle);
      usleep(1000);
      snd_pcm_writei(sp->handle,silence,sizeof(silence) / (2 * sizeof(int16_t)));
      continue;
    }
    size -= r; // stereo samples left (2 16 bit words, 4 bytes)
    outsamps += 2*r; // Each stereo sample is 2 16-bit words
  }
  return 0;
}

struct audio *lookup_session(uint32_t ssrc,struct sockaddr_in *sender){
  struct audio *sp;

  // Walk hash chain
  for(sp = Audio[ssrc & 0xff]; sp != NULL; sp = sp->next){
    if(sp->ssrc == ssrc && memcmp(&sp->sender,sender,sizeof(*sender)) == 0){
      // Found it
      // Move to top of hash chain as we'll probably use it again soon
      if(sp->prev != NULL){
	sp->prev->next = sp->next;
	sp->prev = NULL;
      }
      if(sp->next != NULL)
	sp->next->prev = sp->prev;

      sp->next = Audio[ssrc & 0xff];
      Audio[ssrc & 0xff] = sp;

      sp->lastused = time(NULL); // Update last used time
      return sp;
    }
  }
  return NULL;
}
// Create a new session, partly initialize; ssrc is used as hash key
struct audio *make_session(uint32_t ssrc){
  struct audio *sp;

  if((sp = calloc(1,sizeof(struct audio))) == NULL)
    return NULL; // Shouldn't happen on modern machines!
  
  sp->lastused = time(NULL);
  // Partly initialize entry; caller has to do other fields
  sp->ssrc = ssrc;
  sp->next = Audio[ssrc & 0xff];
  if(sp->next != NULL)
    sp->next->prev = sp;
  Audio[ssrc & 0xff] = sp;
  return sp;
}

int main(int argc,char *argv[]){
  char *mcast_address_string = "239.1.2.6"; // Opus broadcast
  int dport = 5004;

  int c;
  while((c = getopt(argc,argv,"S:I:P:v")) != EOF){
    switch(c){
    case 'v':
      Verbose++;
      break;
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
      fprintf(stderr,"Usage: %s [-v] [-S audioname] [-I mcast_address] [-P dport]\n",argv[0]);
      fprintf(stderr,"Defaults: %s -S %s -I %s -P %d\n",argv[0],Audioname,mcast_address_string,dport);
      exit(1);
    }
  }
  if(Verbose)
    fprintf(stderr,"%s: %s\n",argv[0],opus_get_version_string());


  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);

  // Set up socket, bind to port
  // Add full IPv6 support!
  Input_fd = socket(AF_INET,SOCK_DGRAM,0);
  inet_pton(AF_INET,mcast_address_string,&PCM_mcast_sockaddr.sin_addr);
  PCM_mcast_sockaddr.sin_family = AF_INET;
  PCM_mcast_sockaddr.sin_port = htons(dport);

  if(IN_MULTICAST(ntohl(PCM_mcast_sockaddr.sin_addr.s_addr))){
    // Join only if it's actually a multicast address
    // We could join 0.0.0.0 or our own address to receive unicasts
    // Or 127.0.0.1 for local loopbacks
    struct group_req group_req;
    group_req.gr_interface = 0;
    memcpy(&group_req.gr_group,&PCM_mcast_sockaddr,sizeof(PCM_mcast_sockaddr));
    if(setsockopt(Input_fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0)
      perror("setsockopt ipv4 multicast join");
  }
  // Let other processes also bind to this port
  int reuse = 1;
  if(setsockopt(Input_fd,SOL_SOCKET,SO_REUSEPORT,&reuse,sizeof(reuse)) != 0)
    perror("so_reuseaddr");

  if(bind(Input_fd,(struct sockaddr_in *)&PCM_mcast_sockaddr,sizeof(PCM_mcast_sockaddr)) == -1){
    perror("bind");
    exit(1);
  }
  fprintf(stderr,"Listening on %s:%d\n",mcast_address_string,dport);

  struct iovec iovec[2];
  struct rtp_header rtp;
  int16_t data[Bufsize];
  
  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = data;
  iovec[1].iov_len = sizeof(data);

  struct msghdr message;
  struct sockaddr_in sender;
  message.msg_name = &sender;
  message.msg_namelen = sizeof(sender);
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 2;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  // Flush any pending packets to avoid unnecessary long buffer delays
  int flags;
  
  if((flags = fcntl(Input_fd,F_GETFL)) != -1){
    flags |= O_NONBLOCK;
    if(fcntl(Input_fd,F_SETFL,flags) == 0){
      int flushed = 0;
      while(recvmsg(Input_fd,&message,0) >= 0)
	flushed++;
      flags &= ~O_NONBLOCK;
      fcntl(Input_fd,F_SETFL,flags);
      if(flushed != 0 && Verbose)
	fprintf(stderr,"%d old packets flushed\n",flushed);
    }
  }

  while(1){
    ssize_t size;
    int drop;

    if((size = recvmsg(Input_fd,&message,0)) < sizeof(rtp))
      continue; // Too small to be valid RTP

    // To host order
    rtp.ssrc = ntohl(rtp.ssrc);
    rtp.seq = ntohs(rtp.seq);
    rtp.timestamp = ntohl(rtp.timestamp);

    // Look up session
    struct audio *sp;

    if((sp = lookup_session(rtp.ssrc,&sender)) == NULL){
      // Not found; create new entry
      char src[INET6_ADDRSTRLEN];
      int sport = -1;
      inet_ntop(AF_INET,&sender.sin_addr,src,sizeof(src));      
      sport = ntohs(sender.sin_port);
      fprintf(stderr,"New stream from %s:%d ssrc %x type %d\n",src,sport,
	      rtp.ssrc,rtp.mpt);

      if((sp = make_session(rtp.ssrc)) == NULL){
	fprintf(stderr,"No room!!\n");
	continue;
      }
      // Initialize entry
      sp->name = Audioname;
      audio_init(sp,Samprate,2,L); // sets up audio device, opus encoder
      memcpy(&sp->sender,&sender,sizeof(struct sockaddr_in));
      sp->eseq = rtp.seq;
      sp->etime = rtp.timestamp;
    }
    if(rtp.seq != sp->eseq){
      fprintf(stderr,"%p: expected %d got %d\n",sp,sp->eseq,rtp.seq);
      if((int16_t)(rtp.seq - sp->eseq) < 0){
	sp->eseq = (rtp.seq + 1) & 0xffff;
	continue;	// Drop probable duplicate
      } else
	drop = rtp.seq - sp->eseq; // Apparent # packets dropped
    } else
      drop = 0;
    sp->eseq = (rtp.seq + 1) & 0xffff;

    size -= sizeof(rtp); // Bytes in data
    int16_t outsamps[2*Bufsize];
    int i;
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
	size = opus_decode(sp->opus,NULL,rtp.timestamp - sp->etime,(opus_int16 *)outsamps,sizeof(outsamps),0);
	play_stereo_pcm(sp,outsamps,size);
      }
      size = opus_decode(sp->opus,(unsigned char *)data,size,(opus_int16 *)outsamps,sizeof(outsamps),0);
      break;
    default:
      continue; // ignore
    }
    sp->etime = rtp.timestamp + size;
    play_stereo_pcm(sp,outsamps,size);
  }
  exit(0);
}
