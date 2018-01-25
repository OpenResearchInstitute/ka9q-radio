// $Id: packet.c,v 1.1 2018/01/24 07:04:51 karn Exp karn $
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
int const AL = 1000; // 25 bit times
//int const AM = AN - AL + 1; // should be >= 40, i.e., samprate / bitrate
int const AM = 1049;

extern float Kaiser_beta;

float const Samprate = 48000;
float const Bitrate = 1200;
//int const Samppbit = Samprate/Bitrate;
int const Samppbit = 40;

int Input_fd = -1;





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

  struct filter_out *filter = create_filter_output(sp->filter_in,NULL,1,COMPLEX);
  set_filter(filter,Samprate,+300,+3300,3.0); // Creates analytic, band-limited signal from +900 to +2500 Hz

#if 0

  // Set up sync correlator
  int const sync_length = 9 * Samppbit; // Samples in a sync vector
  float *timebuf = fftwf_alloc_real(1024);
  float complex *fbuf = fftwf_alloc_complex(513);
  fftwf_plan plan = fftwf_plan_dft_r2c_1d(1024,timebuf,fbuf,FFTW_ESTIMATE);
  // Fill timebuf with prototype of flag pattern: 1 bit time of 0, 7 bits of 1, 1 bit of 0
  // This includes the effect of NRZI encoding
  memset(timebuf,0,1024*sizeof(*timebuf));
  int i;
  for(i=0;i<Samppbit;i++)
    timebuf[i] = -1;

  for(;i<8*Samppbit;i++)
    timebuf[i] = +1;

  for(;i<9*Samppbit;i++)
    timebuf[i] = -1;
  
  fftwf_execute(plan); // To frequency domain
  fftwf_destroy_plan(plan);
  fftwf_free(timebuf);
  timebuf = NULL;

  
  struct filter_in *sync_in = create_filter_input(664,sync_length+1,REAL);
  struct filter_out *sync_out = create_filter_output(sync_in,fbuf,1,REAL);




  complex float memory = 0;
  float last_val = 0;
  int samp_since_last_edge = 0;
  unsigned char hdlc_frame[1024];
  memset(hdlc_frame,0,sizeof(hdlc_frame));
  int frame_bit = 0;
  unsigned short crc = 0xffff;
  int zero_pending = 0;
  int sync_corr_count = 0;

#endif
  int sample = 0;
  float complex mark_delay_line[40];
  memset(mark_delay_line,0,sizeof(mark_delay_line));
  int pointer = 0;
  float complex space_delay_line[40];
  memset(space_delay_line,0,sizeof(space_delay_line));
  float complex mark_phase = 1;
  float complex mark_step = csincosf(-2*M_PI*1200./Samprate);
  float complex space_phase = 1;
  float complex space_step = csincosf(-2*M_PI*2200./Samprate);
  float complex mark_sum = 0;
  float complex space_sum = 0;

  printf("signed double\n");


  while(1){
    execute_filter_output(filter);    // Blocks until data appears


    for(int n=0; n<filter->olen; n++){
      float complex s =  mark_phase * filter->output.c[n];
      mark_phase *= mark_step;
#if 0
      if(isnan(crealf(s)) || isnan(cimagf(s)))
	s = 0;
#endif

      mark_sum -= mark_delay_line[pointer];
      mark_sum += mark_delay_line[pointer] = s;

      s =  space_phase * filter->output.c[n];
      space_phase *= space_step;
#if 0
      if(isnan(crealf(s)) || isnan(cimagf(s)))
	s = 0;
#endif

      space_sum -= space_delay_line[pointer];
      space_sum += space_delay_line[pointer] = s;
      pointer = (pointer + 1) % 40;

      float y = cnrmf(mark_sum) - cnrmf(space_sum);

      printf(". %d %f\n",sample++,y);
      
      



#if 0
      // FM demodulate
      float y = cargf(filter->output.c[n] * conjf(memory)) - (2 * M_PI * 1700. / Samprate); // center is 1700 Hz;
      memory = filter->output.c[n];

#if 0

      sync_in->input.r[sync_corr_count++] = y;
      if(sync_corr_count == sync_in->ilen){
	execute_filter_input(sync_in);
	sync_corr_count = 0;
	execute_filter_output(sync_out);
	for(int k=0;k<sync_out->olen;k++){
	  printf(". %d %f\n",sample++,sync_out->output.r[k]);
	}
      }
#endif

#if 0

      if(y * last_val < 0){ // NRZI decode: sign flip = 0, no sign flip = 1
	//	int const crc_poly = 0x1021;
	int const crc_poly = 0x8408;

	// See how many 1 bits we've had since last zero
	// 4 or less: that many 1's followed by zero
	// 5: five 1's, then drop next 0
	// 6: six 1's: flag, end frame
	// 7 or more: abort frame
	int one_bits = roundf((float)samp_since_last_edge / Samppbit) - 1;

	if(one_bits >= 0 && one_bits <= 5){
	  // Previous transition was a zero bit
	  if(zero_pending){
	    // Insert single zero
	    if((crc & 1) ^ 0)
	      crc = (crc >> 1) ^ crc_poly;
	    else
	      crc >>= 1;
	    frame_bit++;
	  }
	  // Insert 'one_bits' ones
	  for(int i=0;i<one_bits;i++){
	    hdlc_frame[frame_bit/8] |= 1 << (frame_bit % 8);
	    if((crc & 1) ^ 1)
	      crc = (crc >> 1) ^ crc_poly;
	    else
	      crc >>= 1;
	    frame_bit++;
	  }
	  if(one_bits < 5)
	    zero_pending = 1;
	  else
	    zero_pending = 0;

	} else if(one_bits == 6) {
	  // Flag
	  // Check CRC, process frame
	  if(frame_bit > 0)
	    printf("FLAG, crc = %x (%s), length = %d bytes %d bits\n",crc,
		   crc == 0xf0b8? "good" : "bad",
		   frame_bit/8,frame_bit%8);

	  if(crc == 0xf0b8){
	    int bytes = frame_bit / 8;
	    for(int i = 0; i < bytes; i++){
	      printf("%02x ",hdlc_frame[i]);
	      if((i % 16) == 15)
		printf("\n");
	    }
	    printf("\n");
	  }

	  memset(hdlc_frame,0,sizeof(hdlc_frame));
	  frame_bit = 0;
	  zero_pending = 0;
	  crc = 0xffff;
	} else if(one_bits >= 7){
	  // Abort
	  memset(hdlc_frame,0,sizeof(hdlc_frame));
	  frame_bit = 0;
	  zero_pending = 0;
	  crc = 0xffff;
	}
	samp_since_last_edge = 0;
      } else {
	// No transition - string of 1 bits
	samp_since_last_edge++;
	// 7 or more consecutive 1's: frame abort
      }
      last_val = y;
#endif
#endif

    }
    mark_phase = mark_phase / cabsf(mark_phase);
    space_phase = space_phase / cabsf(space_phase);

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


  // Initialize these here

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



