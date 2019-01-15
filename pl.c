// $Id: pl.c,v 1.3 2019/01/08 00:27:24 karn Exp karn $
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
#include <locale.h>
#include <signal.h>
#include <getopt.h>

#include "dsp.h"
#include "multicast.h"
#include "osc.h"

// Global config variables
#define MAX_MCAST 20          // Maximum number of multicast addresses
#define PKTSIZE 16384
#define SAMPRATE 48000   // Too hard to handle other sample rates right now
#define PL_BLOCKSIZE (SAMPRATE/4)    // Integration time 250 ms
#define DTMF_BLOCKSIZE (SAMPRATE/20) // Integration time 50 ms

float const SCALE16 = 1./SHRT_MAX;

// Command line params
int Verbose;                  // Verbosity flag (currently unused)
int Mcast_ttl = 10;           // our multicast output is frequently routed
char *Mcast_address_text[MAX_MCAST];

float PL_tones[] = {
     67.0,  69.3,  71.9,  74.4,  77.0,  79.7,  82.5,  85.4,  88.5,  91.5,  94.8,  97.4,
    100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3, 131.8, 136.5, 141.3, 146.2,
    150.0, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2, 192.8, 199.5, 206.5, 213.8,
    221.3, 229.1, 237.1, 245.5, 254.1, 159.8, 165.5, 171.3, 177.3, 183.5, 189.9, 196.6,
    203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 250.3,
};

#define N_tones (sizeof(PL_tones)/sizeof(float))

float DTMF_low_tones[] = { 697, 770, 852, 941 };
float DTMF_high_tones[] = { 1209, 1336, 1477, 1633 };

char DTMF_matrix[4][4] = {   // indexed by [low][high]
  { '1', '2', '3', 'A' },			  
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' },
};


// Global variables
int Nfds;
struct session *Sessions;

struct session {
  struct session *prev;       // Linked list pointers
  struct session *next; 
  int type;                 // input RTP type (10,11)
  
  struct sockaddr sender;
  char addr[NI_MAXHOST];    // RTP Sender IP address
  char port[NI_MAXSERV];    // RTP Sender source port

  struct rtp_state rtp_state_in; // RTP input state

  complex float pl_integrators[N_tones];
  struct osc pl_osc[N_tones];

  complex float dtmf_low_integrators[4];
  complex float dtmf_high_integrators[4];  
  struct osc dtmf_low_osc[4];
  struct osc dtmf_high_osc[4];

  int pl_audio_count;          // Number of samples integrated so far
  int dtmf_audio_count;        // Number of samples integrated so far
};

void closedown(int);
struct session *lookup_session(const struct sockaddr *,uint32_t);
struct session *make_session(struct sockaddr const *r,uint32_t,uint16_t,uint32_t);
int close_session(struct session *);
float process_pl(struct session *sp);
char process_dtmf(struct session *sp);

struct option Options[] =
  {
   {"iface", required_argument, NULL, 'A'},
   {"pcm-in", required_argument, NULL, 'I'},
   {"ttl", required_argument, NULL, 'T'},
   {"verbose", no_argument, NULL, 'v'},
   {NULL, 0, NULL, 0},
};

struct result {
  float energy;
  int index;
};

// greater energy compares as "less" so list is sorted in descending order
int compare(const void *ap,const void *bp){
  const struct result *a = (struct result *)ap;
  const struct result *b = (struct result *)bp;  
  if(a->energy > b->energy)
    return -1;
  else if(a->energy < b->energy)
    return +1;
  return 0;
}


char Optstring[] = "A:I:T:v";

int main(int argc,char * const argv[]){

  setlocale(LC_ALL,getenv("LANG"));

  int c;
  while((c = getopt_long(argc,argv,Optstring,Options,NULL)) != -1){
    switch(c){
    case 'A':
      Default_mcast_iface = optarg;
      break;
    case 'I':
      if(Nfds == MAX_MCAST){
	fprintf(stderr,"Too many multicast addresses; max %d\n",MAX_MCAST);
      } else 
	Mcast_address_text[Nfds++] = optarg;
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
  // Also accept groups without -I option
  for(int i=optind; i < argc; i++){
    if(Nfds == MAX_MCAST){
      fprintf(stderr,"Too many multicast addresses; max %d\n",MAX_MCAST);
    } else 
      Mcast_address_text[Nfds++] = argv[i];
  }
  // Set up multicast
  if(Nfds == 0){
    fprintf(stderr,"Must specify PCM source group(s)\n");
    exit(1);
  }

  // Set up multicast input, create mask for select()
  fd_set fdset_template; // Mask for select()
  FD_ZERO(&fdset_template);
  int max_fd = 2;        // Highest number fd for select()
  int input_fd[Nfds];    // Multicast receive sockets

  for(int i=0;i<Nfds;i++){
    input_fd[i] = setup_mcast(Mcast_address_text[i],NULL,0,0,0);
    if(input_fd[i] == -1){
      fprintf(stderr,"Can't set up input %s\n",Mcast_address_text[i]);
      continue;
    }
    if(input_fd[i] > max_fd)
      max_fd = input_fd[i];
    FD_SET(input_fd[i],&fdset_template);
  }

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);

  char current_dtmf_digit = 0;
  float current_pl_tone = 0;


  while(1){
    // Wait for traffic to arrive
    fd_set fdset = fdset_template;
    int s = select(max_fd+1,&fdset,NULL,NULL,NULL);
    if(s < 0 && errno != EAGAIN && errno != EINTR)
      break;
    if(s == 0)
      continue; // Nothing arrived; probably just an ignored signal

    for(int fd_index = 0;fd_index < Nfds;fd_index++){
      if(input_fd[fd_index] == -1 || !FD_ISSET(input_fd[fd_index],&fdset))
	continue;

      // Receive PCM in RTP/UDP/IP
      struct sockaddr sender;
      unsigned char buffer[PKTSIZE];
      socklen_t socksize = sizeof(sender);
      int size = recvfrom(input_fd[fd_index],buffer,sizeof(buffer),0,&sender,&socksize);
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
      if(size <= 0)
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
      if(sp == NULL && (sp = make_session(&sender,rtp_hdr.ssrc,rtp_hdr.seq,rtp_hdr.timestamp)) == NULL){
	fprintf(stderr,"No room!!\n");
	continue;
      }
      sp->type = rtp_hdr.type;
      int samples_skipped = rtp_process(&sp->rtp_state_in,&rtp_hdr,sampcount);
      if(samples_skipped < 0)
	continue;
      
      short *sampp = (short *)dp;
      while(sampcount-- > 0){
	// For each sample, run the local oscillators and integrators
	float samp = SCALE16 * (short)ntohs(*sampp++);
	for(int n=0; n < N_tones; n++)
	  sp->pl_integrators[n] += samp * step_osc(&sp->pl_osc[n]);
	
	if(++sp->pl_audio_count >= PL_BLOCKSIZE){
	  double pl_tone = process_pl(sp);
	  if(pl_tone != current_pl_tone){
	    if(pl_tone != 0.0)
	      printf("SSRC %x PL %.1f Hz\n",sp->rtp_state_in.ssrc,pl_tone);
	    else
	      printf("SSRC %x PL stop\n",sp->rtp_state_in.ssrc);
	  }
	  current_pl_tone = pl_tone;
	}

	for(int n=0; n < 4; n++){
	  sp->dtmf_low_integrators[n] += samp * step_osc(&sp->dtmf_low_osc[n]);
	  sp->dtmf_high_integrators[n] += samp * step_osc(&sp->dtmf_high_osc[n]);
	}
	if(++sp->dtmf_audio_count >= DTMF_BLOCKSIZE){
	  char dtmf_digit = process_dtmf(sp);
	  if(dtmf_digit != current_dtmf_digit && dtmf_digit != '\0')
	    printf("SSRC %x DTMF %c\n",sp->rtp_state_in.ssrc,dtmf_digit);
	  current_dtmf_digit = dtmf_digit;
	}
      }
    }
  }
}

struct session *lookup_session(const struct sockaddr *sender,const uint32_t ssrc){
  struct session *sp;
  for(sp = Sessions; sp != NULL; sp = sp->next){
    if(sp->rtp_state_in.ssrc == ssrc && memcmp(&sp->sender,sender,sizeof(*sender)) == 0){
      // Found it
      if(sp->prev != NULL){
	// Not at top of bucket chain; move it there
	if(sp->next != NULL)
	  sp->next->prev = sp->prev;

	sp->prev->next = sp->next;
	sp->prev = NULL;
	sp->next = Sessions;
	Sessions = sp;
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
  getnameinfo((struct sockaddr *)sender,sizeof(*sender),sp->addr,sizeof(sp->addr),
	      sp->port,sizeof(sp->port),NI_NOFQDN|NI_DGRAM);

  memcpy(&sp->sender,sender,sizeof(struct sockaddr));
  sp->rtp_state_in.ssrc = ssrc;
  sp->rtp_state_in.seq = seq;
  sp->rtp_state_in.timestamp = timestamp;

  // Set up PL tone steps and phasors
  for(int n=0; n < N_tones; n++){
    sp->pl_integrators[n] = 0;
    set_osc(&sp->pl_osc[n],PL_tones[n]/SAMPRATE,9);
  }

  // Set up DTMF
  for(int n=0; n < 4; n++){
    sp->dtmf_low_integrators[n] = 0;
    sp->dtmf_high_integrators[n] = 0;
    set_osc(&sp->dtmf_low_osc[n],DTMF_low_tones[n]/SAMPRATE,0);
    set_osc(&sp->dtmf_high_osc[n],DTMF_high_tones[n]/SAMPRATE,0);
  }
  // Put at head of bucket chain
  sp->next = Sessions;
  if(sp->next != NULL)
    sp->next->prev = sp;
  Sessions = sp;
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
    Sessions = sp->next;
  free(sp);
  return 0;
}
void closedown(int s){
  while(Sessions != NULL)
    close_session(Sessions);

  exit(0);
}

// Look for PL tone after each integration interval
float process_pl(struct session *sp){
  sp->pl_audio_count = 0;
  
  struct result results[N_tones];
  for(int n=0; n < N_tones; n++){
    results[n].energy = cnrmf(sp->pl_integrators[n]);
    sp->pl_integrators[n] = 0;
    results[n].index = n;
  }
  qsort(results,N_tones,sizeof(results[0]),compare); // Descending energy order
#if 0
  printf("PL debug: sort results\n");
  for(int n=0; n < N_tones; n++)
    printf("%.1f Hz %.1f dB\n",PL_tones[results[n].index],power2dB(results[n].energy));
#endif
  
  float pl_snr = power2dB(results[0].energy) - power2dB(results[1].energy);
  float pl_tone_freq = 0;

  // Tones are not orthogonal so there is some spectral leakage that limits the SNR even with no noise
  if(pl_snr >= 6){
    pl_tone_freq = PL_tones[results[0].index];
#if 0
    printf("PL debug: %s:%s tone %.1f Hz snr %.1f dB\n",sp->addr,sp->port,pl_tone_freq,pl_snr);
#endif
  }
  return pl_tone_freq;
}

// Look for DTMF digit after each integration interval
char process_dtmf(struct session *sp){
  sp->dtmf_audio_count = 0;
  
  int low_tone_index = -1;
  float low_tone_snr = 0;
  {
    float max_energy = 0;
    float total_energy = 0;
    for(int n=0; n < 4; n++){
      float energy = cnrmf(sp->dtmf_low_integrators[n]);
      sp->dtmf_low_integrators[n] = 0;
      total_energy += energy;
      if(energy >= max_energy){
	max_energy = energy;
	low_tone_index = n;
      }
    }
    low_tone_snr = max_energy / (total_energy - max_energy);
    if(low_tone_snr < 10) // 10 dB
      low_tone_index = -1; // Not good enough
  }
  int high_tone_index = -1;
  float high_tone_snr = 0;
  {
    float max_energy = 0;
    float total_energy = 0;
    for(int n=0; n < 4; n++){
      float energy = cnrmf(sp->dtmf_high_integrators[n]);
      sp->dtmf_high_integrators[n] = 0;
      total_energy += energy;
      if(energy >= max_energy){
	max_energy = energy;
	high_tone_index = n;
      }
    }
    high_tone_snr = max_energy / (total_energy - max_energy);
    if(high_tone_snr < 10) // 10 dB
      high_tone_index = -1;
  }
  char result = '\0';
  if(low_tone_index != -1 && high_tone_index != -1){
    result = DTMF_matrix[low_tone_index][high_tone_index];
#if 0
    printf("DTMF debug: %s:%s %c SNR_low %.1f SNR_high %.1f\n",
	   sp->addr,sp->port,
	   result,10*log10(low_tone_snr),10*log10(high_tone_snr));
#endif
  }
  return result;
}
