// $Id: monitor.c,v 1.19 2017/08/25 19:48:37 karn Exp $
// Listen to multicast, send PCM audio to Linux ALSA driver
#define _GNU_SOURCE 1
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <limits.h>
#include <string.h>
#include <opus/opus.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netdb.h>
#if defined(linux)
#include <alsa/asoundlib.h>
#endif

#include "rtp.h"
#include "dsp.h"
#include "multicast.h"

struct audio {
  struct audio *prev; // Linked list pointers
  struct audio *next; 
  uint32_t ssrc;
  int eseq; // Next expected sequence number
  int etime; // Next expected timestamp
  struct sockaddr sender;
  OpusDecoder *opus;
};

// Global variables
char *Mcast_address_text;

// Maximum samples/words per packet
// Bigger than Ethernet MTU because of network device fragmentation/reassembly
const int Bufsize = 8192;
const int L=512;
const int Samprate = 48000; // Too hard to handle other sample rates right now
int Verbose;
#if defined(linux)
char *Audioname = "default";
#else
char *Audioname = NULL; // No ALSA on OSX!
#endif

struct audio *Audio[256]; // Hash chains
int Input_fd = -1;

void close_audio(struct audio *ap){
  if(ap == NULL)
    return;
  
  if(ap->opus != NULL){
    opus_decoder_destroy(ap->opus);
    ap->opus = NULL;
  }
  free(ap);
}


// Only Linux has ALSA
#if defined(linux)

int Underrun;
int Overrun;
snd_pcm_t *Handle = NULL;

// Set up or change ALSA for demodulated sound output
snd_pcm_t *audio_init(const char *name,unsigned samprate,int channels,int L){
  snd_pcm_uframes_t buffer_size;
  snd_pcm_hw_params_t *hw_params;
  snd_pcm_sw_params_t *sw_params;
  snd_pcm_t *handle;

  if(Verbose)
    fprintf(stderr,"audio_init(%s,rate=%u,blksize=%d,chans=%d)\n",name,samprate,L,channels);

  if(snd_pcm_open(&handle,name,SND_PCM_STREAM_PLAYBACK,0) < 0){
    fprintf(stderr,"Error opening D/A %s\n",name);
    return NULL;
  }

  snd_pcm_hw_params_alloca(&hw_params);
  if(snd_pcm_hw_params_any(handle,hw_params) < 0){
    fprintf(stderr,"Can not configure D/A %s\n",name);
    return NULL;
  }
  if(snd_pcm_hw_params_set_access(handle,hw_params,SND_PCM_ACCESS_RW_INTERLEAVED) < 0){
    fprintf(stderr,"Error setting D/A access on %s\n",name);
    return NULL;
  }
  if(snd_pcm_hw_params_set_format(handle,hw_params,SND_PCM_FORMAT_S16_LE)<0){
    fprintf(stderr,"Error setting D/A format on %s\n",name);
    return NULL;
  }
  unsigned int stmp = samprate;
  if(snd_pcm_hw_params_set_rate_near(handle,hw_params,&stmp,0)<0){
    fprintf(stderr,"Error setting D/A sample rate on %s\n",name);
    return NULL;
  }
  if(stmp != samprate)
    fprintf(stderr,"Warning: actual D/A sample rate is %u\n",stmp);

  if(snd_pcm_hw_params_set_channels(handle,hw_params,channels)<0){ // stereo
    fprintf(stderr,"Error setting D/A channels on %s\n",name);
    return NULL;
  }
  // We will generally write L-sample blocks at a time
  snd_pcm_uframes_t period_length = L;
  if(snd_pcm_hw_params_set_period_size_near(handle,hw_params,&period_length,0)<0){
    fprintf(stderr,"Error setting D/A periods on %s\n",name);
    return NULL;
  }
  if(period_length != L)
    fprintf(stderr,"D/A periods: requested %d, got %d\n",(int)L,(int)period_length);

  snd_pcm_uframes_t requested_buffer_size = 65536;
  buffer_size = requested_buffer_size;
  if(snd_pcm_hw_params_set_buffer_size_near(handle,hw_params,&buffer_size)<0){
    fprintf(stderr,"Error setting D/A buffersize on %s\n",name);
    return NULL;
  }
  if(buffer_size != requested_buffer_size)
    fprintf(stderr,"D/A buffer: requested %d, got %d\n",(int)requested_buffer_size,(int)buffer_size);

  if(snd_pcm_hw_params(handle,hw_params)<0){
    fprintf(stderr,"Error setting D/A HW params on %s\n",name);
    return NULL;
  }
  snd_pcm_sw_params_alloca(&sw_params);
  if(snd_pcm_sw_params_current(handle,sw_params) < 0){
    fprintf(stderr,"Can not configure driver for %s\n",name);
    return NULL;
  }
  // !!!
  if(snd_pcm_sw_params_set_start_threshold(handle,sw_params,period_length) < 0){ // One PCM packet
    fprintf(stderr,"Can not configure start threshold for %s\n",name);
    return NULL;
  }
#if 0
  // Does this only apply to capture?
  if(snd_pcm_sw_params_set_stop_threshold(handle,sw_params,8*period_length) < 0){
    fprintf(stderr,"Can not configure stop threshold for %s\n",name);
    return NULL;
  }
#endif
  if(snd_pcm_sw_params(handle,sw_params) < 0)
    fprintf(stderr,"Can not set sw params %s\n",name);

  Underrun = 0;
  Overrun = 0;
  return handle;
}


// play buffer of 'size' samples, each 16 bit stereo (4*size bytes)
int play_stereo_pcm(snd_pcm_t *handle,const int16_t *outsamps,const int size){
  if(size <= 0)
    return 0;

  int r;
  while((r = snd_pcm_writei(handle,outsamps,size)) != size){
    // Seems like mostly underrun errors happen here
    if(r == -EPIPE)
      Underrun++;
    if(Verbose){
      struct timeval tv;
      gettimeofday(&tv,NULL);
      
      fprintf(stderr,"%ld.%06ld snd_pcm_writei fail: %s %d\n",(long)tv.tv_sec,(long)tv.tv_usec,
	      snd_strerror(r),r);
    }
    snd_pcm_prepare(handle);
    int16_t silence[size][2];
    memset(silence,0,sizeof(silence));
    snd_pcm_writei(handle,silence,size);
  }
  return 0;
}
#endif

struct audio *lookup_session(const struct sockaddr *sender,const uint32_t ssrc){

  // Walk hash chain
  struct audio *sp;
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
      return sp;
    }
  }
  return NULL;
}
// Create a new session, partly initialize; ssrc is used as hash key
struct audio *make_session(struct sockaddr *sender,uint32_t ssrc,uint16_t seq,uint32_t timestamp){
  struct audio *sp;

  if((sp = calloc(1,sizeof(*sp))) == NULL)
    return NULL; // Shouldn't happen on modern machines!
  
  // Initialize entry
  memcpy(&sp->sender,sender,sizeof(struct sockaddr));
  sp->ssrc = ssrc;
  sp->eseq = seq;
  sp->etime = timestamp;

  // Put at head of bucket chain
  sp->next = Audio[ssrc & 0xff];
  if(sp->next != NULL)
    sp->next->prev = sp;
  Audio[ssrc & 0xff] = sp;
  return sp;
}

void closedown(){
  exit(0);
}


int main(int argc,char * const argv[]){
  // Note: Audioname defaults to "default" on Linux, "stdout" on OSX (which doesn't have ALSA)
  Mcast_address_text = "239.2.1.1";
  int c;
  while((c = getopt(argc,argv,"S:I:v")) != EOF){
    switch(c){
    case 'v':
      Verbose++;
      break;
#if defined(linux)
    case 'S':
      Audioname = optarg;
      if(strcmp(Audioname,"stdout") == 0)
	Audioname = NULL;  // Special case for standard output
      break;
#endif
    case 'I':
      Mcast_address_text = optarg;
      break;
    default:
#if defined(linux)
      fprintf(stderr,"Usage: %s [-v] [-S audioname] [-I mcast_address]\n",argv[0]);
      fprintf(stderr,"Defaults: %s -S %s -I %s\n",argv[0],Audioname,Mcast_address_text);
#else
      fprintf(stderr,"Usage: %s [-v] [-I mcast_address]\n",argv[0]);
      fprintf(stderr,"Defaults: %s -I %s\n",argv[0],Mcast_address_text);
#endif      
      exit(1);
    }
  }
  if(Verbose)
    fprintf(stderr,"%s: %s\n",argv[0],opus_get_version_string());

  if(Audioname == NULL && isatty(1)){
    fprintf(stderr,"Won't write raw PCM audio to a terminal. Redirect stdout.\n");
    exit(1);
  }
  // Set up multicast input
  Input_fd = setup_mcast(Mcast_address_text,0);
  if(Input_fd == -1){
    fprintf(stderr,"Can't set up input\n");
    exit(1);
  }
  fprintf(stderr,"Listening on %s\n",Mcast_address_text);

#if defined(linux)
  if(Audioname != NULL)
    Handle = audio_init(Audioname,Samprate,2,L); // sets up audio device, opus encoder
#endif

  struct iovec iovec[2];
  struct rtp_header rtp;
  int16_t data[Bufsize];
  
  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = data;
  iovec[1].iov_len = sizeof(data);

  struct msghdr message;
  struct sockaddr sender;
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

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);

  struct timeval receive_time;
  gettimeofday(&receive_time,NULL);
  struct timeval last_receive_time = receive_time;
  int recentbytes = 0;
  float bitrate = 0;

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
    struct audio *sp = lookup_session(&sender,rtp.ssrc);

    if(sp == NULL){
      // Not found; create new entry
      char addr[NI_MAXHOST];
      char port[NI_MAXSERV];
      
      getnameinfo((struct sockaddr *)&sender,sizeof(sender),addr,sizeof(addr),
		    port,sizeof(port),NI_NOFQDN|NI_DGRAM|NI_NUMERICHOST);

      fprintf(stderr,"New stream from %s:%s ssrc %x type %d\n",addr,port,rtp.ssrc,rtp.mpt);

      if((sp = make_session(&sender,rtp.ssrc,rtp.seq,rtp.timestamp)) == NULL){
	fprintf(stderr,"No room!!\n");
	continue;
      }
    }
    int drop = 0;
    if(rtp.seq != sp->eseq){
      int const diff = (int)(rtp.seq - sp->eseq);
      fprintf(stderr,"ssrc %lx: expected %d got %d\n",(unsigned long)rtp.ssrc,sp->eseq,rtp.seq);
      if(diff < 0 && diff > -10)
	continue;	// Drop probable duplicate
      drop = diff; // Apparent # packets dropped
    }
    sp->eseq = (rtp.seq + 1) & 0xffff;
    size -= sizeof(rtp); // Bytes in payload
    recentbytes += size;
    int16_t outsamps[2*Bufsize];
    int i,error;
    int samples = 0;

    switch(rtp.mpt){
    case 10: // Stereo
      samples = size / 4;  // # 32-bit word samples
      for(i=0;i<2*samples;i++)
	outsamps[i] = ntohs(data[i]); // RTP profile specifies big-endian samples
      break;
    case 11: // Mono; send to both stereo channels
      samples = size / 2;
      for(i=0;i<samples;i++)
	outsamps[2*i] = outsamps[2*i+1] = ntohs(data[i]);
      break;
    case 20: // Opus codec decode - arbitrary choice
      if(sp->opus == NULL){ // Create if it doesn't already exist
	sp->opus = opus_decoder_create(Samprate,2,&error);
	if(sp->opus == NULL)
	  fprintf(stderr,"Opus decoder error %d\n",error);
	break;
      }
      if(drop != 0){
	// packet dropped; conceal
	samples = opus_decode(sp->opus,NULL,rtp.timestamp - sp->etime,(opus_int16 *)outsamps,sizeof(outsamps),0);
	if(samples > 0){
#if defined(linux)
	  if(Handle != NULL)
	    play_stereo_pcm(Handle,outsamps,samples);
	  else
#endif
	    write(1,outsamps,samples*2*sizeof(*outsamps));
	}
      }
      samples = opus_decode(sp->opus,(unsigned char *)data,size,(opus_int16 *)outsamps,sizeof(outsamps),0);
      break;
    default:
      continue; // ignore
    }
    sp->etime = rtp.timestamp + samples;
    if(Verbose){
      gettimeofday(&receive_time,NULL);
      long const interval = 1000000. * (receive_time.tv_sec - last_receive_time.tv_sec) + (receive_time.tv_usec - last_receive_time.tv_usec);
      if(interval > 100000){
	bitrate = (1000000. * 8 * recentbytes)/interval;
	last_receive_time = receive_time;
	recentbytes = 0;
      }
      fprintf(stderr,"%06ld usec: ssrc %lx read %d bytes bitrate %'8.1f b/s write %d samps\n",
	      interval,(unsigned long)sp->ssrc,(int)size,bitrate,(int)samples);
    }
    if(samples > 0){
#if defined(linux)
      if(Handle != NULL)
	play_stereo_pcm(Handle,outsamps,samples);
      else
#endif
	write(1,outsamps,samples*2*sizeof(*outsamps));
    }
  }
  exit(0);
}
