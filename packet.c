// $Id$
// AFSK/FM packet demodulator

#define _GNU_SOURCE 1
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <limits.h>
#include <string.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/resource.h>
#include <netdb.h>
#include <locale.h>
#include <errno.h>

#include "filter.h"
#include "misc.h"
#include "multicast.h"

struct packet {
  struct packet *prev;       // Linked list pointers
  struct packet *next; 
  uint32_t ssrc;            // RTP Sending Source ID
  int eseq;                 // Next expected RTP sequence number
  int etime;                // Next expected RTP timestamp
  int type;                 // RTP type (10,11,20)
  
  struct sockaddr sender;
  char addr[NI_MAXHOST];    // RTP Sender IP address
  char port[NI_MAXSERV];    // RTP Sender source port

  int input_pointer;
  struct filter_in *filter_in;
  pthread_t decode_thread;

  unsigned long age;
  unsigned long packets;    // RTP packets for this session
  unsigned long drops;      // Apparent rtp packet drops
  unsigned long invalids;   // Unknown RTP type
  unsigned long empties;    // RTP but no data
  unsigned long dupes;      // Duplicate or old serial numbers
};



char Mcast_address_text[] = "239.2.1.1";
float const SCALE = 1./32768;
int const Bufsize = 2048;
int const AN = 2048; // Should be power of 2 for FFT efficiency
int const AL = 2000; // 50 bit times
//int const AM = AN - AL + 1; // should be >= 40, i.e., samprate / bitrate
int const AM = 49;

extern float Kaiser_beta;

float const Samprate = 48000;
float const Bitrate = 1200;
//int const Samppbit = Samprate/Bitrate;
int const Samppbit = 40;

int Input_fd = -1;

complex float *Mark_response;
complex float *Space_response;




struct packet *Packet;

struct packet *lookup_session(const struct sockaddr *sender,const uint32_t ssrc){
  struct packet *sp;
  for(sp = Packet; sp != NULL; sp = sp->next){
    if(sp->ssrc == ssrc && memcmp(&sp->sender,sender,sizeof(*sender)) == 0){
      // Found it
      if(sp->prev != NULL){
	// Not at top of bucket chain; move it there
	if(sp->next != NULL)
	  sp->next->prev = sp->prev;

	sp->prev->next = sp->next;
	sp->prev = NULL;
	sp->next = Packet;
	Packet = sp;
      }
      return sp;
    }
  }
  return NULL;
}
// Create a new session, partly initialize
struct packet *make_session(struct sockaddr const *sender,uint32_t ssrc,uint16_t seq,uint32_t timestamp){
  struct packet *sp;

  if((sp = calloc(1,sizeof(*sp))) == NULL)
    return NULL; // Shouldn't happen on modern machines!
  
  // Initialize entry
  memcpy(&sp->sender,sender,sizeof(struct sockaddr));
  sp->ssrc = ssrc;
  sp->eseq = seq;
  sp->etime = timestamp;

  // Put at head of bucket chain
  sp->next = Packet;
  if(sp->next != NULL)
    sp->next->prev = sp;
  Packet = sp;
  return sp;
}

int close_session(struct packet *sp){
  if(sp == NULL)
    return -1;
  
  // Remove from linked list
  if(sp->next != NULL)
    sp->next->prev = sp->prev;
  if(sp->prev != NULL)
    sp->prev->next = sp->next;
  else
    Packet = sp->next;
  free(sp);
  return 0;
}

// Packet demod task - incomplete
void *decode_task(void *arg){
  pthread_setname("decode");
  struct packet *sp = (struct packet *)arg;
  assert(sp != NULL);

  struct filter_out *mark_filter = create_filter_output(sp->filter_in,Mark_response,1,COMPLEX);
  struct filter_out *space_filter = create_filter_output(sp->filter_in,Space_response,1,COMPLEX);  
  
#if 0
  printf("signed double\n");

  for(int i = 0;i < AN;i++){
    printf(". %d %f\n",i,cabsf(Space_response[i]));
  }
  exit(0);
#endif



  int sample = 0;

  while(1){
    // These block until data appears
    execute_filter_output(mark_filter);
    execute_filter_output(space_filter);


#if 1
    struct filter_in *filter_in = mark_filter->master;
    printf("signed double\n");
    for(int i=0; 

#endif



#if 0
    // Output unfiltered demodulated waveform at full sample rate for testing
    printf("signed double\n");
    for(int i=0; i < mark_filter->olen; i++){
      float e = cnrmf(mark_filter->output.c[i]) - cnrmf(space_filter->output.c[i]);
      printf(". %d %.0f\n",sample,e);
      sample++;
    }
#endif



#if 0
    // Find symbol timing
    int best_symtime = -1;
    float best_energy = 0;
    for(int symtime = 0; symtime < Samppbit;symtime++){
      // Find total energies at each offset
      float e = 0;
      for(int i=symtime;i<mark_filter->olen;i+= Samppbit)
	e += cnrmf(mark_filter->output.c[i]) + cnrmf(space_filter->output.c[i]);

      if(e > best_energy){
	best_energy = e;
	best_symtime = symtime;
      }
    }
    if(best_symtime == -1){
      // No energy at all; discard
      continue;
    }
#endif

  }



  return NULL;

}


int main(int argc,char *argv[]){


  Mark_response = fftwf_alloc_complex(AN);
  Space_response = fftwf_alloc_complex(AN);



  complex float timedomain[AN];
  fftwf_plan plan = fftwf_plan_dft_1d(AN,timedomain,Mark_response,FFTW_FORWARD,FFTW_ESTIMATE);


  // Initialize these here

  memset(timedomain,0,sizeof(timedomain));
  complex float mark_phase = 1;
  complex float mark_step = csincosf(2 * M_PI * 1200. / Samprate); // Complex sinusoid at +1200 Hz
  for(int i=0;i<Samppbit;i++){ // one bit time of mark tone
    timedomain[i] = mark_phase; 
    mark_phase *= mark_step;
  }


  float kaiser_window[AN];
  Kaiser_beta = 0;
  make_kaiser(kaiser_window,AM,Kaiser_beta);
  for(int i=0;i<AM;i++)
    timedomain[i] *= kaiser_window[i];
  
  fftwf_execute(plan);
  fftwf_destroy_plan(plan);

#if 0
  printf("signed double\n");
  for(int i=0;i<AN;i++){
    printf(". %d %f\n",i,cnrmf(Mark_response[i]));
  }
  exit(0);
#endif

  plan = fftwf_plan_dft_1d(AN,timedomain,Space_response,FFTW_FORWARD,FFTW_ESTIMATE);
  memset(timedomain,0,sizeof(timedomain));
  complex float space_phase = 1;
  complex float space_step = csincosf(2 * M_PI * 2200. / Samprate); // Complex sinusoid at +2200 Hz
  for(int i=0;i<Samppbit;i++){ // one bit time of space tone
    timedomain[i] = space_phase;
    space_phase *= space_step;
  }

  // Reuse kaiser window
  for(int i=0;i<AM;i++)
    timedomain[i] *= kaiser_window[i];



  fftwf_execute(plan);
  fftwf_destroy_plan(plan);

  // Set up multicast input
  Input_fd = setup_mcast(Mcast_address_text,0);
  if(Input_fd == -1){
    fprintf(stderr,"Can't set up input\n");
    exit(1);
  }
  struct iovec iovec[2];
  struct rtp_header rtp;
  //  int16_t data[Bufsize];
  signed short data[Bufsize];
  
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

  // audio input thread
  // Receive audio multicasts, multiplex into sessions, execute filter front end (which wakes up decoder thread)
  while(1){
    int size;

    size = recvmsg(Input_fd,&message,0);
    if(size == -1){
      if(errno != EINTR){ // Happens routinely
	perror("recvmsg");
	usleep(1000);
      }
      continue;
    }
    if(size < sizeof(rtp)){
      usleep(500); // Avoid tight loop
      continue; // Too small to be valid RTP
    }
    // To host order
    rtp.ssrc = ntohl(rtp.ssrc);
    rtp.seq = ntohs(rtp.seq);
    rtp.timestamp = ntohl(rtp.timestamp);

    if(rtp.mpt != 10 && rtp.mpt != 20 && rtp.mpt != 11) // 1 byte, no need to byte swap
      goto endloop; // Discard unknown RTP types to avoid polluting session table


    struct packet *sp = lookup_session(&sender,rtp.ssrc);
    if(sp == NULL){
      // Not found
      if((sp = make_session(&sender,rtp.ssrc,rtp.seq,rtp.timestamp)) == NULL){
	fprintf(stderr,"No room!!\n");
	goto endloop;
      }
      getnameinfo((struct sockaddr *)&sender,sizeof(sender),sp->addr,sizeof(sp->addr),
		  //		    sp->port,sizeof(sp->port),NI_NOFQDN|NI_DGRAM|NI_NUMERICHOST);
		    sp->port,sizeof(sp->port),NI_NOFQDN|NI_DGRAM);
      sp->dupes = 0;
      sp->age = 0;
      sp->input_pointer = 0;
      sp->filter_in = create_filter_input(AL,AM,REAL);
      pthread_create(&sp->decode_thread,NULL,decode_task,sp); // One decode thread per stream
      fprintf(stderr,"New session from %s, ssrc %x\n",sp->addr,sp->ssrc);
    }
    sp->age = 0;
    int drop = 0;

    sp->packets++;
    if(rtp.seq != sp->eseq){
      int const diff = (int)(rtp.seq - sp->eseq);
      //        fprintf(stderr,"ssrc %lx: expected %d got %d\n",(unsigned long)rtp.ssrc,sp->eseq,rtp.seq);
      if(diff < 0 && diff > -10){
	sp->dupes++;
	goto endloop;	// Drop probable duplicate
      }
      drop = diff; // Apparent # packets dropped
      sp->drops += abs(drop);
    }
    sp->eseq = (rtp.seq + 1) & 0xffff;

    sp->type = rtp.mpt;
    size -= sizeof(rtp); // Bytes in payload
    if(size <= 0){
      sp->empties++;
      goto endloop; // empty?!
    }
    int samples = 0;

    switch(rtp.mpt){
    case 11: // Mono only for now
      samples = size / 2;
      signed short *dp = data;
      while(samples-- > 0){
	// Swap sample to host order, convert to float

	sp->filter_in->input.r[sp->input_pointer++] = ntohs(*dp++) * SCALE;
	if(sp->input_pointer == sp->filter_in->ilen){
	  execute_filter_input(sp->filter_in); // Wakes up any threads waiting for data on this filter
	  sp->input_pointer = 0;
	}
      }
      break;
    default:
      samples = 0;
      break; // ignore
    }
    sp->etime = rtp.timestamp + samples;

  endloop:;
  }
  // Need to kill decoder threads? Or will ordinary signals reach them?
  exit(0);
}



