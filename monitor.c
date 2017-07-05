// $Id: monitor.c,v 1.6 2017/07/04 09:47:43 karn Exp karn $
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
#include <sys/time.h>

#include "rtp.h"
#include "dsp.h"

// Maximum samples/words per packet
// Bigger than Ethernet MTU because of network device fragmentation/reassembly
const int Bufsize = 8192;
#define Silence_size 7680 // 160 ms @ 48 kHz // Largest Opus frame??
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
  snd_pcm_uframes_t period_length = L;
  if(snd_pcm_hw_params_set_period_size_near(ap->handle,hw_params,&period_length,0)<0){
    fprintf(stderr,"Error setting D/A periods on %s\n",ap->name);
    return -1;
  }
  if(period_length != L)
    fprintf(stderr,"D/A periods: requested %d, got %d\n",(int)L,(int)period_length);

  snd_pcm_uframes_t requested_buffer_size = 65536;
  buffer_size = requested_buffer_size;
  if(snd_pcm_hw_params_set_buffer_size_near(ap->handle,hw_params,&buffer_size)<0){
    fprintf(stderr,"Error setting D/A buffersize on %s\n",ap->name);
    return -1;
  }
  if(buffer_size != requested_buffer_size)
    fprintf(stderr,"D/A buffer: requested %d, got %d\n",(int)requested_buffer_size,(int)buffer_size);

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
  if(snd_pcm_sw_params_set_start_threshold(ap->handle,sw_params,period_length) < 0){ // One PCM packet
    fprintf(stderr,"Can not configure start threshold for %s\n",ap->name);
    return -1;
  }
#if 0
  // Does this only apply to capture?
  if(snd_pcm_sw_params_set_stop_threshold(ap->handle,sw_params,8*period_length) < 0){
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
int play_stereo_pcm(struct audio * const sp,const int16_t *outsamps,const int size){
  static struct timeval lastcall; // Time of last call
  
  if(Verbose > 1){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    if(lastcall.tv_sec == 0 && lastcall.tv_usec == 0)
      lastcall = tv; // Avoid garbage on first call
    fprintf(stderr,"%ld.%06ld (%06ld usec) ssrc %lx write %d samps\n",
	    (long)tv.tv_sec,(long)tv.tv_usec,1000000*(tv.tv_sec - lastcall.tv_sec) + (tv.tv_usec - lastcall.tv_usec),
	    (unsigned long)sp->ssrc,size);
    lastcall = tv;
  }

  int r;
  if((r = snd_pcm_writei(sp->handle,outsamps,size)) != size){
    // Seems like mostly underrun errors happen here
    if(Verbose){
      struct timeval tv;
      gettimeofday(&tv,NULL);
      
      fprintf(stderr,"%ld.%06ld ssrc %lx snd_pcm_writei fail: %s %d\n",(long)tv.tv_sec,(long)tv.tv_usec,
	      (long unsigned int)sp->ssrc,snd_strerror(r),r);
    }
    snd_pcm_prepare(sp->handle);
    int16_t silence[2*size][2];
    memset(silence,0,sizeof(silence));
    snd_pcm_writei(sp->handle,silence,2*size);
  }
  return 0;
}

struct audio *lookup_session(const uint32_t ssrc,const struct sockaddr_in *sender){
  struct audio *sp;

  // Walk hash chain
  for(sp = Audio[ssrc & 0xff]; sp != NULL; sp = sp->next){
    if(sp->ssrc == ssrc && memcmp(&sp->sender,sender,sizeof(*sender)) == 0){
      // Found it
      if(sp->prev != NULL){
	// Not at top of bucket chain; move it there
	if(sp->next != NULL)
	  sp->next->prev = sp->prev;

	sp->prev->next = sp->next;
	sp->prev = NULL;
	sp->next = Audio[ssrc & 0xff];
	Audio[ssrc & 0xff] = sp;
      }
      sp->lastused = time(NULL); // Update last used time
      return sp;
    }
  }
  return NULL;
}
// Create a new session, partly initialize; ssrc is used as hash key
struct audio *make_session(const uint32_t ssrc){
  struct audio *sp;

  if((sp = calloc(1,sizeof(struct audio))) == NULL)
    return NULL; // Shouldn't happen on modern machines!
  
  sp->lastused = time(NULL);
  // Partly initialize entry; caller has to do other fields
  // Put at head of bucket chain
  sp->ssrc = ssrc;
  sp->next = Audio[ssrc & 0xff];
  if(sp->next != NULL)
    sp->next->prev = sp;
  Audio[ssrc & 0xff] = sp;
  return sp;
}

int main(int argc,char *argv[]){
  char *mcast_address_string = "239.2.1.1";
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
      dport = strtol(optarg,NULL,0);
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

    if((size = recvmsg(Input_fd,&message,0)) < sizeof(rtp)){
      usleep(500); // Avoid tight loop
      perror("recvmsg");
      continue; // Too small to be valid RTP
    }
    // To host order
    rtp.ssrc = ntohl(rtp.ssrc);
    rtp.seq = ntohs(rtp.seq);
    rtp.timestamp = ntohl(rtp.timestamp);

    // Look up session
    struct audio *sp;

    if((sp = lookup_session(rtp.ssrc,&sender)) == NULL){
      // Not found; create new entry
      char src[INET6_ADDRSTRLEN];
      inet_ntop(AF_INET,&sender.sin_addr,src,sizeof(src));      
      int sport = ntohs(sender.sin_port);
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
    int drop = 0;
    if(rtp.seq != sp->eseq){
      int diff = (int)(rtp.seq - sp->eseq);
      fprintf(stderr,"ssrc %lx: expected %d got %d\n",(unsigned long)rtp.ssrc,sp->eseq,rtp.seq);
      if(diff < 0 && diff > -10){
	continue;	// Drop probable duplicate
      }
      drop = diff; // Apparent # packets dropped
    }
    sp->eseq = (rtp.seq + 1) & 0xffff;

    size -= sizeof(rtp); // Bytes in data
    int16_t outsamps[2*Bufsize];
    int i,error;
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

      if(sp->opus == NULL){ // Create if it doesn't already exist
	sp->opus = opus_decoder_create(sp->samprate,2,&error);
	if(sp->opus == NULL)
	  fprintf(stderr,"Opus decoder error %d\n",error);
	break;
      }
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
