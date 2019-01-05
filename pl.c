// $Id$
// PL tone decoder
// Reads multicast PCM audio (mono only right now)
// Copyright Jan 2019 Phil Karn, KA9Q
#define _GNU_SOURCE 1
#include <assert.h>
#include <errno.h>
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <netdb.h>
#include <locale.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <signal.h>
#include <fftw3.h>
#include <getopt.h>

#include "dsp.h"
#include "misc.h"
#include "multicast.h"
#include "osc.h"

// Global config variables
#define Pktsize 16384
#define Samprate 48000   // Too hard to handle other sample rates right now
#define Bufsize (Samprate/4)    // Integration time

float const SCALE16 = 1./SHRT_MAX;

// Command line params
char *Mcast_input_address_text;     // Multicast address we're listening to
char *Mcast_output_address_text;    // Multicast address we're sending to
int Verbose;                  // Verbosity flag (currently unused)
int Mcast_ttl = 10;           // our multicast output is frequently routed

// Global variables
int Input_fd = -1;            // Multicast receive socket
int Output_fd = -1;           // Multicast receive socket
struct session *Audio;

struct session {
  struct session *prev;       // Linked list pointers
  struct session *next; 
  int type;                 // input RTP type (10,11)
  
  struct sockaddr sender;
  char addr[NI_MAXHOST];    // RTP Sender IP address
  char port[NI_MAXSERV];    // RTP Sender source port

  struct rtp_state rtp_state_in; // RTP input state

  int audio_count;          // Number of samples integrated so far
};

void closedown(int);
struct session *lookup_session(const struct sockaddr *,uint32_t);
struct session *make_session(struct sockaddr const *r,uint32_t,uint16_t,uint32_t);
int close_session(struct session *);
float analyze(struct session *sp);

struct option Options[] =
  {
   {"iface", required_argument, NULL, 'A'},
   {"pcm-in", required_argument, NULL, 'I'},
   {"ttl", required_argument, NULL, 'T'},
   {"verbose", no_argument, NULL, 'v'},
   {NULL, 0, NULL, 0},
  };

char Optstring[] = "A:I:T:v";

float PL_tones[] = {
     67.0,  69.3,  71.9,  74.4,  77.0,  79.7,  82.5,  85.4,  88.5,  91.5,  94.8,  97.4,
    100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3, 131.8, 136.5, 141.3, 146.2,
    150.0, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2, 192.8, 199.5, 206.5, 213.8,
    221.3, 229.1, 237.1, 245.5, 254.1, 159.8, 165.5, 171.3, 177.3, 183.5, 189.9, 196.6,
    203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 250.3, };

#define N_tones (sizeof(PL_tones)/sizeof(float))
struct osc PL_osc[N_tones];

float DTMF_low_tones[] = { 697, 770, 852, 941 };
struct osc DTMF_low_osc[4];
float DTMF_high_tones[] = { 1209, 1336, 1477, 1633 };
struct osc DTMF_high_osc[4];

char DTMF_matrix[4][4] = {   // indexed by [low][high]
  { '1', '2', '3', 'A' },			  
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' },
};


int main(int argc,char * const argv[]){

  setlocale(LC_ALL,getenv("LANG"));

  int c;
  Mcast_ttl = 10;

  while((c = getopt_long(argc,argv,Optstring,Options,NULL)) != -1){
    switch(c){
    case 'A':
      Default_mcast_iface = optarg;
      break;
    case 'I':
      Mcast_input_address_text = optarg;
      break;
    case 'T':
      Mcast_ttl = strtol(optarg,NULL,0);
      break;
    case 'v':
      Verbose++;
      break;
    default:
      break;
    }
  }
  // Set up multicast
  if(!Mcast_input_address_text){
    fprintf(stderr,"Must specify -I option\n");
    exit(1);
  }

  Input_fd = setup_mcast(Mcast_input_address_text,NULL,0,0,0);
  if(Input_fd == -1){
    fprintf(stderr,"Can't set up input on %s: %sn",Mcast_input_address_text,strerror(errno));
    exit(1);
  }
#if 0
  Output_fd = setup_mcast(Mcast_output_address_text,NULL,1,Mcast_ttl,0);
  if(Output_fd == -1){
    fprintf(stderr,"Can't set up output on %s: %s\n",Mcast_output_address_text,strerror(errno));
    exit(1);
  }
#endif

  // Set up to receive PCM in RTP/UDP/IP
  struct sockaddr sender;

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);

  // Set up PL tone steps and phasors
  complex float PL_integrators[N_tones];
  memset(PL_integrators,0,sizeof(PL_integrators));
  for(int n=0; n < N_tones; n++)
    set_osc(&PL_osc[n],PL_tones[n]/Samprate,9);

  // Set up DTMF
  complex float DTMF_low_integrators[4];
  complex float DTMF_high_integrators[4];  
  memset(&DTMF_low_integrators,0,sizeof(DTMF_low_integrators));
  memset(&DTMF_high_integrators,0,sizeof(DTMF_high_integrators));
  for(int n=0; n < 4; n++){
    set_osc(&DTMF_low_osc[n],DTMF_low_tones[n]/Samprate,0);
    set_osc(&DTMF_high_osc[n],DTMF_high_tones[n]/Samprate,0);
  }

  while(1){
    unsigned char buffer[Pktsize];
    socklen_t socksize = sizeof(sender);
    int size = recvfrom(Input_fd,buffer,sizeof(buffer),0,&sender,&socksize);
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

    int sampcount;
    switch(rtp_hdr.type){
    case PCM_MONO_PT:
      sampcount = size / sizeof(short);
      break;
    default:
      continue; // Discard all but mono PCM to avoid polluting session table
    }

    struct session *sp = lookup_session(&sender,rtp_hdr.ssrc);
    if(sp == NULL){
      // Not found
      if((sp = make_session(&sender,rtp_hdr.ssrc,rtp_hdr.seq,rtp_hdr.timestamp)) == NULL){
	fprintf(stderr,"No room!!\n");
	continue;
      }
      getnameinfo((struct sockaddr *)&sender,sizeof(sender),sp->addr,sizeof(sp->addr),
		    sp->port,sizeof(sp->port),NI_NOFQDN|NI_DGRAM);
    }
    sp->type = rtp_hdr.type;
    int samples_skipped = rtp_process(&sp->rtp_state_in,&rtp_hdr,sampcount);
    if(samples_skipped < 0)
      continue;
    
    short *sampp = (short *)dp;
    sp->audio_count += sampcount;
    while(sampcount-- > 0){
      float samp = SCALE16 * (short)ntohs(*sampp++);
      for(int n=0; n < N_tones; n++)
	PL_integrators[n] += samp * step_osc(&PL_osc[n]);
      for(int n=0; n < 4; n++){
	DTMF_low_integrators[n] += samp * step_osc(&DTMF_low_osc[n]);
	DTMF_high_integrators[n] += samp * step_osc(&DTMF_high_osc[n]);
      }
    }
    if(sp->audio_count >= Bufsize){
      sp->audio_count = 0;

      // Look for PL tone
      int pl_tone = -1;
      float pl_snr = 0;
      {
	float maxenergy = 0;
	float totenergy = 0;
	for(int n=0; n < N_tones; n++){
	  float energy = cnrmf(PL_integrators[n]);
	  totenergy += energy;
	  if(energy >= maxenergy){
	    maxenergy = energy;
	    pl_tone = n;
	  }
	}
	memset(PL_integrators,0,sizeof(PL_integrators)); // Reset integrators
	pl_snr = maxenergy / (totenergy - maxenergy);
	if(pl_snr < 2) // 3 dB
	  pl_tone = -1; // Not good enough
	if(pl_tone != -1)
	  printf("SSRC %x PL %.1f Hz %.1f dB total %.1f dB SNR %.1f dB\n",
		 sp->rtp_state_in.ssrc,
		 PL_tones[pl_tone],
		 10*log10f(maxenergy),
		 10*log10f(totenergy),
		 10*log10f(maxenergy/(totenergy - maxenergy))
		 );

      }

      // Look for DTMF
      int lowtone = -1;
      float lowsnr = 0;
      {
	float maxenergy = 0;
	float totenergy = 0;
	for(int n=0; n < 4; n++){
	  float energy = cnrmf(DTMF_low_integrators[n]);
	  totenergy += energy;
	  if(energy >= maxenergy){
	    maxenergy = energy;
	    lowtone = n;
	  }
	}
	memset(DTMF_low_integrators,0,sizeof(DTMF_low_integrators));
	lowsnr = maxenergy / (totenergy - maxenergy);
	if(lowsnr < 4) // 6 dB
	  lowtone = -1; // Not good enough
      }
      int hightone = -1;
      float highsnr = 0;
      {
	float maxenergy = 0;
	float totenergy = 0;
	for(int n=0; n < 4; n++){
	  float energy = cnrmf(DTMF_high_integrators[n]);
	  totenergy += energy;
	  if(energy >= maxenergy){
	    maxenergy = energy;
	    hightone = n;
	  }
	}
	memset(DTMF_high_integrators,0,sizeof(DTMF_high_integrators));
	highsnr = maxenergy / (totenergy - maxenergy);
	if(highsnr < 4) // 6 dB
	  hightone = -1;
      }
      if(lowtone != -1 && hightone != -1)
	printf("DTMF %c SNR_low %.1f SNR_high %.1f\n",
	       DTMF_matrix[lowtone][hightone],
	       10*log10(lowsnr),
	       10*log10(highsnr));
    }
  }
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
