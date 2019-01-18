// $Id: opus.c,v 1.34 2019/01/08 06:48:18 karn Exp karn $
// Opus compression relay
// Read PCM audio from one multicast group, compress with Opus and retransmit on another
// Currently subject to memory leaks as old group states aren't yet aged out
// Copyright Jan 2018 Phil Karn, KA9Q
#define _GNU_SOURCE 1
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <opus/opus.h>
#include <netdb.h>
#include <locale.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <signal.h>
#include <getopt.h>
#include <pthread.h>

#include "misc.h"
#include "multicast.h"
#include "status.h"

struct session {
  struct session *prev;       // Linked list pointers
  struct session *next; 
  int type;                 // input RTP type (10,11)
  
  struct sockaddr sender;
  char addr[NI_MAXHOST];    // RTP Sender IP address
  char port[NI_MAXSERV];    // RTP Sender source port

  struct rtp_state rtp_state_in; // RTP input state
  OpusEncoder *opus;        // Opus encoder handle
  int silence;              // Currently suppressing silence

  float *audio_buffer;      // Buffer to accumulate PCM until enough for Opus frame
  int audio_index;          // Index of next sample to write into audio_buffer

  struct rtp_state rtp_state_out; // RTP output state

  unsigned long underruns;  // Callback count of underruns (stereo samples) replaced with silence
};


// Global config variables
int const Bufsize = 16384;     // Maximum samples/words per RTP packet - must be bigger than Ethernet MTU
int const Samprate = 48000;   // Too hard to handle other sample rates right now
                              // Opus will notice the actual audio bandwidth, so there's no real cost to this

int const Channels = 2;       // Stereo - no penalty if the audio is actually mono, Opus will figure it out
float const SCALE = 1./SHRT_MAX;

// Command line params
int Verbose;                  // Verbosity flag (currently unused)
int Opus_bitrate = 32;        // Opus stream audio bandwidth; default 32 kb/s
int Discontinuous = 0;        // Off by default
float Opus_blocktime = 20;    // 20 ms, a reasonable default
int Fec = 0;                  // Use forward error correction
int Mcast_ttl = 10;           // our multicast output is frequently routed

// Global variables
int Status_fd = -1;           // Reading from radio status
int Status_out_fd = -1;       // Writing to radio status
int Input_fd = -1;            // Multicast receive socket
int Output_fd = -1;           // Multicast receive socket
int Opus_frame_size;
struct session *Audio;
struct state State[256];
uint64_t Output_packets;


void closedown(int);
struct session *lookup_session(const struct sockaddr *,uint32_t);
struct session *make_session(struct sockaddr const *r,uint32_t,uint16_t,uint32_t);
int close_session(struct session *);
int send_samples(struct session *sp,float left,float right);
void *input(void *arg);

struct option Options[] =
  {
   {"iface", required_argument, NULL, 'A'},
   {"block-time", required_argument, NULL, 'B'},
   {"pcm-in", required_argument, NULL, 'I'},
   {"status-in", required_argument, NULL, 'S'},
   {"opus-out", required_argument, NULL, 'R'},
   {"ttl", required_argument, NULL, 'T'},
   {"fec", required_argument, NULL, 'f'},
   {"bitrate", required_argument, NULL, 'o'},
   {"verbose", no_argument, NULL, 'v'},
   {"discontinuous", no_argument, NULL, 'x'},
   {NULL, 0, NULL, 0},
  };
   
char Optstring[] = "A:B:I:R:S:T:f:o:vx";

struct sockaddr_storage Status_dest_address;
struct sockaddr_storage Status_input_source_address;
struct sockaddr_storage Local_status_source_address;
struct sockaddr_storage PCM_dest_address;
struct sockaddr_storage PCM_source_address;
struct sockaddr_storage Opus_dest_address;
struct sockaddr_storage Opus_source_address;

struct sockcache SC; // TEMP


int main(int argc,char * const argv[]){

  setlocale(LC_ALL,getenv("LANG"));

  int c;
  while((c = getopt_long(argc,argv,Optstring,Options,NULL)) != -1){
    switch(c){
    case 'A':
      Default_mcast_iface = optarg;
      break;
    case 'B':
      Opus_blocktime = strtod(optarg,NULL);
      break;
    case 'I':
      Input_fd = setup_mcast(optarg,(struct sockaddr *)&PCM_dest_address,0,0,0);
      if(Input_fd == -1)
	fprintf(stderr,"Can't set up input on %s: %sn",optarg,strerror(errno));

      break;
    case 'R':
      Output_fd = setup_mcast(optarg,(struct sockaddr *)&Opus_dest_address,1,Mcast_ttl,0);
      if(Output_fd == -1)
	fprintf(stderr,"Can't set up output on %s: %s\n",optarg,strerror(errno));
      {
	socklen_t len = sizeof(Opus_source_address);
	getsockname(Output_fd,(struct sockaddr *)&Opus_source_address,&len);
      }
      break;
    case 'S':
      if(Status_fd != -1){
	fprintf(stderr,"warning: only last --status-in is used\n");
	close(Status_fd);
	Status_fd = -1;
	if(Status_out_fd != -1){
	  close(Status_out_fd);
	  Status_out_fd = -1;
	}
      }
      Status_fd = setup_mcast(optarg,(struct sockaddr *)&Status_dest_address,0,0,2);
      if(Status_fd == -1){
	fprintf(stderr,"Can't set up input on %s: %s\n",optarg,strerror(errno));
	exit(1);
      }
      Status_out_fd = setup_mcast(NULL,(struct sockaddr *)&Status_dest_address,1,Mcast_ttl,2);
      {
	socklen_t len;
	len = sizeof(Local_status_source_address);
	getsockname(Status_out_fd,(struct sockaddr *)&Local_status_source_address,&len);
      }
      break;
    case 'T':
      Mcast_ttl = strtol(optarg,NULL,0);
      break;
    case 'f':
      Fec = strtol(optarg,NULL,0);
      break;
    case 'o':
      Opus_bitrate = strtol(optarg,NULL,0);
      break;
    case 'v':
      Verbose++;
      break;
    case 'x':
      Discontinuous = 1;
      break;
    default:
      fprintf(stderr,"Usage: %s [-x] [-v] [-o bitrate] [-B blocktime] [-T mcast_ttl] -I input_mcast_address -R output_mcast_address\n",argv[0]);
      fprintf(stderr,"Defaults: %s -o %d -B %.1f -I (none) -R (none) -T %d\n",argv[0],Opus_bitrate,Opus_blocktime,Mcast_ttl);
      exit(1);
    }
  }
  if(Opus_blocktime != 2.5 && Opus_blocktime != 5
     && Opus_blocktime != 10 && Opus_blocktime != 20
     && Opus_blocktime != 40 && Opus_blocktime != 60
     && Opus_blocktime != 80 && Opus_blocktime != 100
     && Opus_blocktime != 120){
    fprintf(stderr,"opus block time must be 2.5/5/10/20/40/60/80/100/120 ms\n");
    fprintf(stderr,"80/100/120 supported only on opus 1.2 and later\n");
    exit(1);
  }
  Opus_frame_size = round(Opus_blocktime * Samprate / 1000.);
  if(Opus_bitrate < 500)
    Opus_bitrate *= 1000; // Assume it was given in kb/s

  // Set up multicast
  if(Input_fd == -1 && Status_fd == -1){
    fprintf(stderr,"Must specify either --status-in or --pcm-in\n");
    exit(1);
  }
  if(Output_fd == -1){
    fprintf(stderr,"Must specify --opus-out\n");
    exit(1);
  }

  // Set up to receive PCM in RTP/UDP/IP
  pthread_t input_thread;
  if(Input_fd != -1)
    pthread_create(&input_thread,NULL,input,NULL);

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);

  // Radio status channel reception and transmission

  while(Status_fd == -1)
    sleep(1); // Status channel not specified

  int full_status_counter = 0;
  while(1){
    socklen_t socklen = sizeof(Status_input_source_address);
    unsigned char buffer[16384];
    int length = recvfrom(Status_fd,buffer,sizeof(buffer),0,(struct sockaddr *)&Status_input_source_address,&socklen);

    // We MUST ignore our own status packets, or we'll loop!
    if(memcmp(&Status_input_source_address, &Local_status_source_address, sizeof(Local_status_source_address)) == 0)
      continue;

    if(length <= 0){
      usleep(10000);
      continue;
    }
    // Parse entries
    {
      int cr = buffer[0];
      if(cr == 1)
	continue; // Ignore commands
      unsigned char *cp = buffer+1;

      while(cp - buffer < length){
	enum status_type type = *cp++;
	
	if(type == EOL)
	  break;
	
	unsigned int optlen = *cp++;
	if(cp - buffer + optlen > length)
	  break;
	
	switch(type){
	case EOL:
	  goto done;
	case OUTPUT_DATA_DEST_SOCKET:
	  decode_socket(&PCM_dest_address,cp,optlen);
	  update_sockcache(&SC,(struct sockaddr *)&PCM_dest_address);
	  if(Input_fd == -1){
	    if(Verbose){
	      struct sockcache sc;
	      update_sockcache(&sc,(struct sockaddr *)&PCM_dest_address);
	      fprintf(stderr,"Listening for PCM on %s:%s\n",sc.host,sc.port);
	    }
	    Input_fd = setup_mcast(NULL,(struct sockaddr *)&PCM_dest_address,0,0,0);
	    if(Input_fd != -1)
	      pthread_create(&input_thread,NULL,input,NULL);
	  }
	  break;
	default:  // Ignore all others for now
	  break;
	}
	cp += optlen;
      }
    done:;
    }
    // Announce ourselves
    {
      unsigned char packet[2048],*bp;
      memset(packet,0,sizeof(packet));
      bp = packet;
      *bp++ = 0; // Response (not a command)
      encode_socket(&bp,OPUS_SOURCE_SOCKET,&Opus_source_address);
      encode_socket(&bp,OPUS_DEST_SOCKET,&Opus_dest_address);
      encode_int(&bp,OPUS_BITRATE,Opus_bitrate);
      encode_int(&bp,OPUS_PACKETS,Output_packets);
      encode_int(&bp,OPUS_TTL,Mcast_ttl);
      // Add more later
      encode_eol(&bp);
      int len = compact_packet(&State[0],packet,full_status_counter == 0);
      if(full_status_counter-- <= 0)
	full_status_counter = 10;
      if(len > 2)
	send(Status_out_fd,packet,len,0);
    }
  }
}



void *input(void *arg){
  if(Verbose)
    fprintf(stderr,"input thread running\n");

  while(1){
    unsigned char buffer[Bufsize];
    socklen_t socksize = sizeof(PCM_source_address);
    int size = recvfrom(Input_fd,buffer,sizeof(buffer),0,(struct sockaddr *)&PCM_source_address,&socksize);
    if(size == -1){
      if(errno != EINTR){ // Happens routinely
	perror("recvfrom");
	usleep(1000);
      }
      continue;
    }
    if(size <= RTP_MIN_SIZE){
      usleep(500); // Avoid tight loop
      continue; // Too small to be valid RTP
    }
    // RTP header to host format
    struct rtp_header rtp_hdr;
    unsigned char *dp = ntoh_rtp(&rtp_hdr,buffer);
    size -= (dp - buffer);
    if(rtp_hdr.pad){
      // Remove padding
      size -= dp[size-1];
      rtp_hdr.pad = 0;
    }
    if(size < 0)
      continue; // Bogus RTP header?

    int frame_size = 0;
    switch(rtp_hdr.type){
    case PCM_STEREO_PT:
      frame_size = size / (2 * sizeof(short));
      break;
    case PCM_MONO_PT:
      frame_size = size / sizeof(short);
      break;
    default:
      goto endloop; // Discard all but mono and stereo PCM to avoid polluting session table
    }

    struct session *sp = lookup_session((struct sockaddr *)&PCM_source_address,rtp_hdr.ssrc);
    if(sp == NULL){
      // Not found
      if((sp = make_session((struct sockaddr *)&PCM_source_address,rtp_hdr.ssrc,rtp_hdr.seq,rtp_hdr.timestamp)) == NULL){
	fprintf(stderr,"No room!!\n");
	goto endloop;
      }
      getnameinfo((struct sockaddr *)&PCM_source_address,sizeof(PCM_source_address),sp->addr,sizeof(sp->addr),
		    sp->port,sizeof(sp->port),NI_NOFQDN|NI_DGRAM);
      sp->audio_buffer = malloc(Channels * sizeof(float) * Opus_frame_size);
      sp->audio_index = 0;
      sp->rtp_state_out.ssrc = rtp_hdr.ssrc;
      int error = 0;
      sp->opus = opus_encoder_create(Samprate,Channels,OPUS_APPLICATION_AUDIO,&error);
      if(error != OPUS_OK || !sp->opus){
	fprintf(stderr,"opus_encoder_create error %d\n",error);
	exit(1);
      }
      error = opus_encoder_ctl(sp->opus,OPUS_SET_DTX(Discontinuous));
      if(error != OPUS_OK)
	fprintf(stderr,"opus_encoder_ctl set discontinuous %d: error %d\n",Discontinuous,error);

      error = opus_encoder_ctl(sp->opus,OPUS_SET_BITRATE(Opus_bitrate));
      if(error != OPUS_OK)
	fprintf(stderr,"opus_encoder_ctl set bitrate %d: error %d\n",Opus_bitrate,error);

      if(Fec){
	error = opus_encoder_ctl(sp->opus,OPUS_SET_INBAND_FEC(1));
	if(error != OPUS_OK)
	  fprintf(stderr,"opus_encoder_ctl set FEC on error %d\n",error);
	error = opus_encoder_ctl(sp->opus,OPUS_SET_PACKET_LOSS_PERC(Fec));
	if(error != OPUS_OK)
	  fprintf(stderr,"opus_encoder_ctl set FEC loss rate %d%% error %d\n",Fec,error);
      }

      // Always seems to return error -5 even when OK??
      error = opus_encoder_ctl(sp->opus,OPUS_FRAMESIZE_ARG,Opus_blocktime);
      if(0 && error != OPUS_OK)
	fprintf(stderr,"opus_encoder_ctl set framesize %d (%.1lf ms): error %d\n",Opus_frame_size,Opus_blocktime,error);
    }
    sp->type = rtp_hdr.type;
    int samples_skipped = rtp_process(&sp->rtp_state_in,&rtp_hdr,frame_size);
    if(samples_skipped < 0)
      goto endloop; // Old dupe
    
    if(rtp_hdr.marker || samples_skipped > 4*Opus_frame_size){
      // reset encoder state after 4 frames of complete silence or a RTP marker bit
      opus_encoder_ctl(sp->opus,OPUS_RESET_STATE);
      sp->silence = 1;
    }
    int sampcount = 0;
    signed short *samples = (signed short *)dp;
    switch(rtp_hdr.type){
    case PCM_STEREO_PT: // Stereo
      sampcount = size / 4;  // # 32-bit word samples
      for(int i=0; i < sampcount; i++){
	float left = SCALE * (signed short)ntohs(samples[2*i]);
	float right = SCALE * (signed short)ntohs(samples[2*i+1]);
	send_samples(sp,left,right);
      }
      break;
    case PCM_MONO_PT: // Mono; send to both stereo channels
      sampcount = size / 2;
      for(int i=0;i<sampcount;i++){
	float left = SCALE * (signed short)ntohs(samples[i]);
	float right = left;
	send_samples(sp,left,right);
      }
      break;
    default:
      sampcount = 0;
      break; // ignore
    }

  endloop:;
  }
  exit(0);
}

struct session *lookup_session(const struct sockaddr *sender,const uint32_t ssrc){
  struct session *sp;
  for(sp = Audio; sp != NULL; sp = sp->next){
    if(sp->rtp_state_in.ssrc == ssrc && memcmp(&sp->sender,sender,sizeof(*sender)) == 0){
      // Found it
      if(sp->prev != NULL){
	// Not at top of bucket chain; move it there
	if(sp->next != NULL)
	  sp->next->prev = sp->prev;

	sp->prev->next = sp->next;
	sp->prev = NULL;
	sp->next = Audio;
	Audio = sp;
      }
      return sp;
    }
  }
  return NULL;
}
// Create a new session, partly initialize
struct session *make_session(struct sockaddr const *sender,uint32_t ssrc,uint16_t seq,uint32_t timestamp){
  struct session *sp;

  if((sp = calloc(1,sizeof(*sp))) == NULL)
    return NULL; // Shouldn't happen on modern machines!
  
  // Initialize entry
  memcpy(&sp->sender,sender,sizeof(struct sockaddr));
  sp->rtp_state_in.ssrc = ssrc;
  sp->rtp_state_in.seq = seq;
  sp->rtp_state_in.timestamp = timestamp;

  // Put at head of bucket chain
  sp->next = Audio;
  if(sp->next != NULL)
    sp->next->prev = sp;
  Audio = sp;
  return sp;
}

int close_session(struct session *sp){
  if(sp == NULL)
    return -1;
  
  if(sp->opus != NULL){
    opus_encoder_destroy(sp->opus);
    sp->opus = NULL;
  }
  if(sp->audio_buffer)
    free(sp->audio_buffer);
  sp->audio_buffer = NULL;

  // Remove from linked list
  if(sp->next != NULL)
    sp->next->prev = sp->prev;
  if(sp->prev != NULL)
    sp->prev->next = sp->next;
  else
    Audio = sp->next;
  free(sp);
  return 0;
}
void closedown(int s){
  while(Audio != NULL)
    close_session(Audio);

  exit(0);
}
// Enqueue a stereo pair of samples for transmit, encode and send Opus
// frame when we have enough
int send_samples(struct session *sp,float left,float right){
  int size = 0;
  sp->audio_buffer[sp->audio_index++] = left;
  sp->audio_buffer[sp->audio_index++] = right;  
  if(sp->audio_index >= Opus_frame_size * Channels){
    sp->audio_index = 0;

    // Set up to transmit Opus RTP/UDP/IP
    struct rtp_header rtp_hdr;
    memset(&rtp_hdr,0,sizeof(rtp_hdr));
    rtp_hdr.version = RTP_VERS;
    rtp_hdr.type = OPUS_PT; // Opus
    rtp_hdr.ssrc = sp->rtp_state_out.ssrc;
    rtp_hdr.seq = sp->rtp_state_out.seq;

    if(sp->silence){
      // Beginning of talk spurt after silence, set marker bit
      rtp_hdr.marker = 1;
      sp->silence = 0;
    } else
      rtp_hdr.marker = 0;

    rtp_hdr.timestamp = sp->rtp_state_out.timestamp;
    sp->rtp_state_out.timestamp += Opus_frame_size; // Always increase timestamp
    
    unsigned char outbuffer[Bufsize];
    unsigned char *dp = outbuffer;
    dp = hton_rtp(dp,&rtp_hdr);
    size = opus_encode_float(sp->opus,sp->audio_buffer,Opus_frame_size,dp,sizeof(outbuffer) - (dp - outbuffer));
    dp += size;
    if(!Discontinuous || size > 2){
      // ship it
      if(send(Output_fd,outbuffer,dp-outbuffer,0) < 0)
	return -1;
      Output_packets++; // all sessions
      sp->rtp_state_out.seq++; // Increment only if packet is sent
      sp->rtp_state_out.bytes += size;
      sp->rtp_state_out.packets++;
    } else
      sp->silence = 1;
  }
  return size;
}
