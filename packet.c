// $Id: packet.c,v 1.30 2019/01/07 00:07:46 karn Exp karn $
// AFSK/FM packet demodulator
// Reads RTP PCM audio stream, emits decoded frames in multicast RTP
// Copyright 2018, Phil Karn, KA9Q

#define _GNU_SOURCE 1
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <locale.h>
#include <netdb.h>
#include <getopt.h>

#include "dsp.h"
#include "osc.h"
#include "filter.h"
#include "misc.h"
#include "multicast.h"
#include "ax25.h"
#include "status.h"

// Needs to be redone with common RTP receiver module
struct session {
  struct session *next; 
  
  struct sockcache source;

  struct rtp_state rtp_state_in;
  struct rtp_state rtp_state_out;

  int input_pointer;
  struct filter_in *filter_in;
  pthread_t decode_thread;
  unsigned int decoded_packets;
};

// Config constants
#define MAX_MCAST 20          // Maximum number of multicast addresses
float const SCALE = 1./32768;
int const PKTSIZE = 16384;
int const AN = 2048; // Should be power of 2 for FFT efficiency
int const AL = 1000; // 25 bit times
//int const AM = AN - AL + 1; // should be >= Samppbit, i.e., samprate / bitrate
int const AM = 1049;
float const Samprate = 48000;
float const Bitrate = 1200;
//int const Samppbit = Samprate/Bitrate;
int const Samppbit = 40;

// Command line params
int Verbose;
int Mcast_ttl = 10;           // Very low intensity output

// Global variables
int Nfds;          // Number of PCM streams
fd_set Fdset_template; // Mask for select()
int Max_fd = 2;        // Highest number fd for select()
int Input_fd[MAX_MCAST];    // Multicast receive sockets
pthread_t Input_thread;

int Output_fd = -1;
int Status_fd = -1;
int Status_out_fd = -1; // Not used yet
struct session *Session;
pthread_mutex_t Output_mutex;
struct sockaddr_storage Status_dest_address;
struct sockaddr_storage Status_input_source_address;
struct sockaddr_storage Local_status_source_address;
struct sockaddr_storage PCM_dest_address; // From incoming status messages (max 1)

struct session *lookup_session(const uint32_t ssrc);
struct session *make_session(uint32_t ssrc);
int close_session(struct session *sp);
void *input(void *arg);
void *decode_task(void *arg);

struct option Options[] =
  {
   {"iface", required_argument, NULL, 'A'},
   {"pcm-in", required_argument, NULL, 'I'},
   {"ax25-out", required_argument, NULL, 'R'},
   {"status-in", required_argument, NULL, 'S'},
   {"ttl", required_argument, NULL, 'T'},
   {"verbose", no_argument, NULL, 'v'},
   {NULL, 0, NULL, 0},
  };
char Optstring[] = "A:I:R:S:T:v";


int main(int argc,char *argv[]){
  // Drop root if we have it
  if(seteuid(getuid()) != 0)
    fprintf(stderr,"seteuid: %s\n",strerror(errno));

  setlocale(LC_ALL,getenv("LANG"));
  FD_ZERO(&Fdset_template);
  // Unlike aprs and aprsfeed, stdout is not line buffered because each packet
  // generates a multi-line dump. So we have to be sure to fflush(stdout) after each
  // packet in case we're redirected into a file

  int c;
  while((c = getopt_long(argc,argv,Optstring,Options,NULL)) != EOF){
    switch(c){
    case 'A':
      Default_mcast_iface = optarg;
      break;
    case 'I':
      if(Nfds == MAX_MCAST){
	fprintf(stderr,"Too many multicast addresses; max %d\n",MAX_MCAST);
	break;
      }
      Input_fd[Nfds] = setup_mcast(optarg,NULL,0,0,0);
      if(Input_fd[Nfds] == -1){
	fprintf(stderr,"Can't set up input %s\n",optarg);
	break;
      }
      Max_fd = max(Max_fd,Input_fd[Nfds]);
      FD_SET(Input_fd[Nfds],&Fdset_template);
      Nfds++;
      if(Status_fd != -1)
	fprintf(stderr,"warning: --status-in ignored when --pcm-in specified\n");
      break;
    case 'R':
      Output_fd = setup_mcast(optarg,NULL,1,Mcast_ttl,0);
      break;
    case 'S':
      if(Nfds != 0){
	fprintf(stderr,"--status-in ignored when --pcm-in specified\n");
	break;
      }
      if(Status_fd != -1){
	fprintf(stderr,"Warning: only last --status-in is used\n");
	close(Status_fd);
	Status_fd = -1;
      }
      Status_fd = setup_mcast(optarg,(struct sockaddr *)&Status_dest_address,0,0,2);
      if(Status_fd == -1){
	fprintf(stderr,"Can't set up status input on %s: %s\n",optarg,strerror(errno));
	exit(1);
      }
#if 0 // Later use?
      Status_out_fd = setup_mcast(NULL,(struct sockaddr *)&Status_dest_address,1,Mcast_ttl,2);
      {
	socklen_t len;
	len = sizeof(Local_status_source_address);
	getsockname(Status_out_fd,(struct sockaddr *)&Local_status_source_address,&len);
      }
#endif
      break;
    case 'T':
      Mcast_ttl = strtol(optarg,NULL,0);
      break;
    case 'v':
      Verbose++;
      break;
    default:
      fprintf(stderr,"Usage: %s [--verbose|-v] [--ttl|-T mcast_ttl] [--pcm-in|-I input_mcast_address [--pcm-in|-I address2]] [--ax25-out|-R output_mcast_address] [input_address ...]\n",argv[0]);
      exit(1);
    }
  }
  if(Output_fd == -1){
    fprintf(stderr,"Must specify --ax25-out\n");
    exit(1);
  }

  // Also accept groups without -I option
  for(int i=optind; i < argc; i++){
    if(Nfds == MAX_MCAST){
      fprintf(stderr,"Too many multicast addresses; max %d\n",MAX_MCAST);
      break;
    }
    Input_fd[Nfds] = setup_mcast(argv[i],NULL,0,0,0);
    if(Input_fd[Nfds] == -1){
      fprintf(stderr,"Can't set up input %s\n",argv[i]);
      continue;
    }
    Max_fd = max(Max_fd,Input_fd[Nfds]);
    FD_SET(Input_fd[Nfds],&Fdset_template);
    Nfds++;
    if(Status_fd != -1)
      fprintf(stderr,"warning: --status-in ignored when --pcm-in specified\n");
  }

  if(Nfds == 0  && Status_fd == -1){
    fprintf(stderr,"Must specify either --status-in or --pcm-in\n");
    exit(1);
  }

  pthread_mutex_init(&Output_mutex,NULL);

  if(Nfds > 0)
    pthread_create(&Input_thread,NULL,input,NULL);

  while(Status_fd == -1)
    sleep(10000); // Status channel not specified; sleep indefinitely

  // Process status messages that may tell us the PCM input
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
	  if(Nfds == 0){
	    // For now, process at most one source in status messages only if not explicitly given with --pcm-in
	    if(Verbose){
	      struct sockcache sc;
	      update_sockcache(&sc,(struct sockaddr *)&PCM_dest_address);
	      fprintf(stderr,"joining pcm input channel %s:%s\n",sc.host,sc.port);
	    }

	    Input_fd[Nfds] = setup_mcast(NULL,(struct sockaddr *)&PCM_dest_address,0,0,0);
	    if(Input_fd[Nfds] != -1){
	      Max_fd = max(Max_fd,Input_fd[Nfds]);
	      FD_SET(Input_fd[Nfds],&Fdset_template);
	      Nfds++;
	      pthread_create(&Input_thread,NULL,input,NULL);
	    }
	  }
	  break;
	default:  // Ignore all others for now
	  break;
	}
	cp += optlen;
      }
    done:;
    }
  }
}

// Process input PCM
void *input(void *arg){

  // audio input thread
  // Receive audio multicasts, multiplex into sessions, execute filter front end (which wakes up decoder thread)

  struct rtp_header rtp_hdr;
  struct sockaddr sender;

  while(1){
    // Wait for traffic to arrive
    fd_set fdset = Fdset_template;
    int s = select(Max_fd+1,&fdset,NULL,NULL,NULL);
    if(s < 0 && errno != EAGAIN && errno != EINTR)
      break;
    if(s == 0)
      continue; // Nothing arrived; probably just an ignored signal

    for(int fd_index = 0;fd_index < Nfds;fd_index++){
      if(Input_fd[fd_index] == -1 || !FD_ISSET(Input_fd[fd_index],&fdset))
	continue;

      unsigned char buffer[PKTSIZE];
      socklen_t socksize = sizeof(sender);
      int size = recvfrom(Input_fd[fd_index],buffer,sizeof(buffer),0,&sender,&socksize);
      if(size == -1){
	if(errno != EINTR){ // Happens routinely
	  perror("recvfrom");
	  usleep(1000); // avoid tight loop
	}
	continue;
      }
      if(size < RTP_MIN_SIZE)
	continue; // Too small to be valid RTP

      // Extract RTP header
      unsigned char *dp = buffer;
      dp = ntoh_rtp(&rtp_hdr,dp);
      size -= dp - buffer;
      
      if(rtp_hdr.pad){
	// Remove padding
	size -= dp[size-1];
	rtp_hdr.pad = 0;
      }
      if(size < 0)
	continue; // garbled RTP header?
      
      if(rtp_hdr.type != PCM_MONO_PT)
	continue; // Only mono PCM for now
      
      struct session *sp = lookup_session(rtp_hdr.ssrc);
      if(sp == NULL){
	// Not found
	if((sp = make_session(rtp_hdr.ssrc)) == NULL){
	  fprintf(stdout,"No room for new session!!\n");
	  fflush(stdout);
	  continue;
	}
	sp->rtp_state_out.ssrc = sp->rtp_state_in.ssrc = rtp_hdr.ssrc;
	sp->input_pointer = 0;
	sp->filter_in = create_filter_input(AL,AM,REAL);
	pthread_create(&sp->decode_thread,NULL,decode_task,sp); // One decode thread per stream
	if(Verbose){
	  update_sockcache(&sp->source,&sender); // Not needed except for verbose debugging
	  fprintf(stdout,"New session from %s:%s, ssrc %x\n",sp->source.host,sp->source.port,sp->rtp_state_in.ssrc);
	  fflush(stdout);
	}
      }
      int sample_count = size / sizeof(signed short); // 16-bit sample count
      int skipped_samples = rtp_process(&sp->rtp_state_in,&rtp_hdr,sample_count);
      if(skipped_samples < 0)
	continue;	// Drop probable duplicate(s)
      
      // Ignore skipped_samples > 0; no real need to maintain sample count when squelch closes
      // Even if its caused by dropped RTP packets there's no FEC to fix it anyway
      signed short *samples = (signed short *)dp;
      while(sample_count-- > 0){
	// Swap sample to host order, convert to float
	sp->filter_in->input.r[sp->input_pointer++] = ntohs(*samples++) * SCALE;
	if(sp->input_pointer == sp->filter_in->ilen){
	  execute_filter_input(sp->filter_in); // Wakes up any threads waiting for data on this filter
	  sp->input_pointer = 0;
	}
      }
    }
  }
  return NULL; // Never gets here
}



// Find existing session in table, if it exists
struct session *lookup_session(const uint32_t ssrc){
  struct session *sp;
  for(sp = Session; sp != NULL; sp = sp->next){
    if(sp->rtp_state_in.ssrc == ssrc)
      // Found it
      return sp;
  }
  return NULL;
}
// Create a new session, partly initialize
struct session *make_session(uint32_t ssrc){
  struct session *sp;

  if((sp = calloc(1,sizeof(*sp))) == NULL)
    return NULL; // Shouldn't happen on modern machines!
  
  sp->rtp_state_in.ssrc = ssrc;

  // Put at head of bucket chain
  sp->next = Session;
  Session = sp;
  return sp;
}

int close_session(struct session *sp){
  if(sp == NULL)
    return -1;
  
  // Remove from linked list
  struct session *se,*se_prev = NULL;
  for(se = Session; se && se != sp; se_prev = se,se = se->next)
    ;
  if(!se)
    return -1;
  
  if(se == sp){
    if(se_prev)
      se_prev->next = sp->next;
    else
      Session = se_prev;
  }
  return 0;
}

// AFSK demod, HDLC decode
void *decode_task(void *arg){
  pthread_setname("afsk");
  struct session *sp = (struct session *)arg;
  assert(sp != NULL);

  struct filter_out *filter = create_filter_output(sp->filter_in,NULL,1,COMPLEX);
  set_filter(filter,+100./Samprate,+4000./Samprate,3.0); // Creates analytic, band-limited signal

  // Tone replica generators (-1200 and -2200 Hz)
  struct osc mark;
  memset(&mark,0,sizeof(mark));
  pthread_mutex_init(&mark.mutex,NULL);
  set_osc(&mark,-1200./Samprate, 0.0);
  
  struct osc space;
  memset(&space,0,sizeof(space));
  pthread_mutex_init(&space.mutex,NULL);
  set_osc(&space,-2200./Samprate, 0.0);  
    
  // Tone integrators
  int symphase = 0;
  float complex mark_accum = 0; // On-time
  float complex space_accum = 0;
  float complex mark_offset_accum = 0; // Straddles previous zero crossing
  float complex space_offset_accum = 0;
  float last_val = 0;  // Last on-time symbol
  float mid_val = 0;   // Last zero crossing symbol

  // hdlc state
  unsigned char hdlc_frame[1024];
  memset(hdlc_frame,0,sizeof(hdlc_frame));
  int frame_bit = 0;
  int flagsync = 0;
  int ones = 0;

  while(1){
    execute_filter_output(filter,0);    // Blocks until data appears

    for(int n=0; n<filter->olen; n++){

      // Spin down by 1200 and 2200 Hz, accumulate each in boxcar (comb) filters
      // Mark and space each have in-phase and offset integrators for timing recovery
      float complex s;
      s = filter->output.c[n] * step_osc(&mark);
      mark_accum += s;
      mark_offset_accum += s;

      s = filter->output.c[n] * step_osc(&space);
      space_accum += s;
      space_offset_accum += s;

      if(++symphase == Samppbit/2){
	// Finish offset integrator and reset
	mid_val = cnrmf(mark_offset_accum) - cnrmf(space_offset_accum);
	mark_offset_accum = space_offset_accum = 0;
      }
      if(symphase < Samppbit)
	continue;
      
      // Finished whole bit
      symphase = 0;
      float cur_val = cnrmf(mark_accum) - cnrmf(space_accum);
      mark_accum = space_accum = 0;

      assert(frame_bit >= 0);
      if(cur_val * last_val < 0){
	// Transition -- Gardner-style clock adjust
	symphase += ((cur_val - last_val) * mid_val) > 0 ? +1 : -1;

	// NRZI zero
	if(ones == 6){
	  // Flag
	  if(flagsync){
	    frame_bit -= 7; // Remove 0111111
	    int bytes = frame_bit / 8;
	    if(bytes > 0 && crc_good(hdlc_frame,bytes)){
	      if(Verbose){
		time_t t;
		struct tm *tmp;
		time(&t);
		tmp = gmtime(&t);
		// Lock output to prevent intermingled output
		pthread_mutex_lock(&Output_mutex);

		fprintf(stdout,"%d %s %04d %02d:%02d:%02d UTC ",tmp->tm_mday,Months[tmp->tm_mon],tmp->tm_year+1900,
		       tmp->tm_hour,tmp->tm_min,tmp->tm_sec);
		
		fprintf(stdout,"ssrc %x packet %d len %d:\n",sp->rtp_state_in.ssrc,sp->decoded_packets++,bytes);
		dump_frame(stdout,hdlc_frame,bytes);
		fflush(stdout);
		pthread_mutex_unlock(&Output_mutex);
	      }
	      struct rtp_header rtp_hdr;
	      memset(&rtp_hdr,0,sizeof(rtp_hdr));
	      rtp_hdr.version = 2;
	      rtp_hdr.type = AX25_PT;
	      rtp_hdr.seq = sp->rtp_state_out.seq++;
	      // RTP timestamp??
	      rtp_hdr.timestamp = sp->rtp_state_out.timestamp;
	      sp->rtp_state_out.timestamp += bytes;
	      rtp_hdr.ssrc = sp->rtp_state_out.ssrc;

	      unsigned char packet[2048],*dp;
	      dp = packet;
	      dp = hton_rtp(dp,&rtp_hdr);
	      memcpy(dp,hdlc_frame,bytes);
	      dp += bytes;
	      send(Output_fd,packet,dp - packet,0); // Check return code?
	      sp->rtp_state_out.packets++;
	      sp->rtp_state_out.bytes += bytes;
	    }
	  }
	  if(1 || frame_bit != 0){
	    memset(hdlc_frame,0,sizeof(hdlc_frame));
	    frame_bit = 0;
	  }
	  flagsync = 1;
	} else if(ones == 5){
	  // Drop stuffed zero
	} else if(ones < 5){
	  if(flagsync){
	    frame_bit++;
	  }
	}
	ones = 0;
      } else {
	// NRZI one
	if(++ones == 7){
	  // Abort
	  if(1 || frame_bit != 0){
	    memset(hdlc_frame,0,sizeof(hdlc_frame));
	    frame_bit = 0;
	  }
	  flagsync = 0;
	} else {
	  if(flagsync){
	    hdlc_frame[frame_bit/8] |= 1 << (frame_bit % 8);
	    frame_bit++;
	  }
	}
      }
      last_val = cur_val;
    }
  }

  return NULL;

}


