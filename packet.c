// $Id: packet.c,v 1.2 2018/01/25 20:05:08 karn Exp karn $
// AFSK/FM packet demodulator

#define _GNU_SOURCE 1
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <locale.h>
#include <errno.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netdb.h>

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



char *Mcast_address_text = "audio-pcm-mcast.local";
float const SCALE = 1./32768;
int const Bufsize = 2048;
int const AN = 2048; // Should be power of 2 for FFT efficiency
int const AL = 1000; // 25 bit times
//int const AM = AN - AL + 1; // should be >= Samppbit, i.e., samprate / bitrate
int const AM = 1049;
float const Samprate = 48000;
float const Bitrate = 1200;
//int const Samppbit = Samprate/Bitrate;
int const Samppbit = 40;

int Input_fd = -1;
struct packet *Packet;
extern float Kaiser_beta;
int Verbose;

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


int dump_frame(unsigned char *frame,int bytes){

  // By default, no digipeaters; will update if there are any
  unsigned char *control = frame + 14;

  // Source address
  // Is this the transmitter?
  int this_transmitter = 1;
  int digipeaters = 0;
  // Look for digipeaters
  if(!(frame[13] & 1)){
    // Scan digipeater list; have any repeated it?
    for(int i=0;i<8;i++){
      unsigned char digi_ssid = frame[20 + 7*i];
      
      digipeaters++;
      if(digi_ssid & 0x80){
	// Yes, passed this one, keep looking
	this_transmitter = 2 + i;
      }
      if(digi_ssid & 1)
	break; // Last digi
    }
  }
  // Show source address, in upper case if this is the transmitter, otherwise lower case
  for(int n=7; n < 13; n++){
    unsigned char c = frame[n] >> 1;
    if(c == ' ')
      break;
    if(this_transmitter == 1)
      putchar(toupper(c));
    else
      putchar(tolower(c));
  }
  printf("-%d",(frame[13] >> 1) & 0xf); // SSID
  
  printf(" -> ");
  
  // List digipeaters

  if(!(frame[13] & 0x1)){
    // Digipeaters are present
    for(int i=0; i<digipeaters; i++){
      for(int k=0;k<6;k++){
	unsigned char c = frame[14 + 7*i + k] >> 1;
	if(c == ' ')
	  break;
	
	if(this_transmitter == 2+i)
	  putchar(toupper(c));
	else
	  putchar(tolower(c));
      }
      int ssid = frame[14 + 7*i + 6];
      printf("-%d",(ssid>> 1) & 0xf); // SSID
      printf(" -> ");
      if(ssid  & 0x1){ // Last one
	control = frame + 14 + 7*i + 7;
	break;
      }
    }
  }
  // NOW print destination, in lower case since it's not the transmitter
  for(int n=0; n < 6; n++){
    unsigned char c = frame[n] >> 1;
    if(c == ' ')
      break;
    putchar(tolower(c));
  }
  printf("-%d",(frame[6] >> 1) & 0xf); // SSID

  // Type field
  printf("; control = %02x",*control++);
  printf("; type = %02x\n",*control);

  for(int i = 0; i < bytes; i++){
    printf("%02x ",frame[i]);
    if((i % 16) == 15 || i == bytes-1){
      for(int k = (i % 16); k < 15; k++)
	printf("   "); // blanks as needed in last line

      printf(" |  ");
      for(int k=(i & ~0xf );k <= i; k++){
	if(isprint(frame[k]))
	  putchar(frame[k]);
	else
	  putchar('.');
      }
      printf("\n");
    }
  }
  printf("\n");
  return 0;
}

// Check CRC on frame
int crc_good(unsigned char *frame,int length){
  unsigned int const crc_poly = 0x8408;
	
  unsigned short crc = 0xffff;
  while(length-- > 0){
    unsigned char byte = *frame++;
    for(int i=0; i < 8; i++){
      unsigned short feedback = 0;
      if((crc ^ byte) & 1)
	feedback = crc_poly;

      crc = (crc >> 1) ^ feedback;
      byte >>= 1;
    }
  }
  if(crc == 0xf0b8)
    return 1;
  else
    return 0;
}



// Packet demod task - incomplete
void *decode_task(void *arg){
  pthread_setname("decode");
  struct packet *sp = (struct packet *)arg;
  assert(sp != NULL);

  struct filter_out *filter = create_filter_output(sp->filter_in,NULL,1,COMPLEX);
  set_filter(filter,Samprate,+100,+4000,3.0); // Creates analytic, band-limited signal

  int bits_since_last_edge = 0;
  unsigned char hdlc_frame[1024];
  memset(hdlc_frame,0,sizeof(hdlc_frame));
  int frame_bit = 0;
  int zero_pending = 0;
  float last_val = 0;
  int packets = 0;


  float complex mark_delay_line[Samppbit];
  memset(mark_delay_line,0,sizeof(mark_delay_line));
  int pointer = 0;
  float complex space_delay_line[Samppbit];
  memset(space_delay_line,0,sizeof(space_delay_line));
  float complex mark_phase = 1;
  float complex mark_step = csincosf(-2*M_PI*1200./Samprate);
  float complex space_phase = 1;
  float complex space_step = csincosf(-2*M_PI*2200./Samprate);
  float complex mark_sum = 0;
  float complex space_sum = 0;
  int flagsync = 0;
#if 0
  int sample = 0;
  printf("signed double\n");
#endif

  int ones = 0;
  int zeroes = 0;

  float peak_mark = 0;
  float peak_space = 0;
  float const peak_smooth = .001;
  float old_y[Samppbit];
  memset(old_y,0,sizeof(old_y));

  while(1){
    execute_filter_output(filter);    // Blocks until data appears

    float y[filter->olen];
    float phase_energies[Samppbit];

    memset(phase_energies,0,sizeof(phase_energies));
    int pointer_start = pointer;
    for(int n=0; n<filter->olen; n++){

      // Spin down by 1200 and 2200 Hz, accumulate each in a sliding boxcar (comb) filter
      float complex s =  mark_phase * filter->output.c[n];
      mark_phase *= mark_step;
      mark_sum -= mark_delay_line[pointer]; // Remove old sample falling off back end of boxcar
      mark_sum += mark_delay_line[pointer] = s; // And add in the new one
	peak_mark *= (1-peak_smooth);
      if(cnrmf(mark_sum) >= peak_mark)
	peak_mark = cnrmf(mark_sum);


      s =  space_phase * filter->output.c[n];
      space_phase *= space_step;
      space_sum -= space_delay_line[pointer];
      space_sum += space_delay_line[pointer] = s;
      peak_space *= (1-peak_smooth);
      if(cnrmf(space_sum) >= peak_space)
	peak_space = cnrmf(space_sum);

      // Noncoherent FSK detection: which tone channel has more energy?
      y[n] = cnrmf(mark_sum) - cnrmf(space_sum);
      phase_energies[pointer] += fabsf(y[n]);
      
      if(++pointer >= Samppbit)
	pointer = 0;
    }
    mark_phase = mark_phase / cabsf(mark_phase);
    space_phase = space_phase / cabsf(space_phase);

    // Which phase is best?
    float max_signal = 0;
    int max_n = -1;
    for(int n=0;n < Samppbit;n++){
      if(phase_energies[n] >= max_signal){
	max_signal = phase_energies[n];
	max_n = n;
      }
    }

    // Now actually process the demodulated data
    for(int i = max_n - pointer_start; i < filter->olen; i += Samppbit){
      float yval;

      if(i < 0)
	yval = old_y[i + Samppbit];
      else
	yval = y[i];

#if 1
      assert(frame_bit >= 0);
      if(yval * last_val < 0){
	// NRZI zero
	if(ones == 6){
	  // Flag
	  if(flagsync){
	    frame_bit -= 7; // Remove 0111111
	    int bytes = frame_bit / 8;
	    if(bytes > 0 && crc_good(hdlc_frame,bytes)){
	      printf("Packet %d, %d bytes\n",packets++,bytes);
	      dump_frame(hdlc_frame,bytes);
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
#else

      if(yval * last_val < 0){ // NRZI decode: sign flip = 0, no sign flip = 1

	// See how many 1 bits we've had since last zero
	// 4 or less: that many 1's followed by zero
	// 5: five 1's, then drop next 0
	// 6: six 1's: flag, end frame
	// 7 or more: abort frame
	int one_bits = bits_since_last_edge;
	bits_since_last_edge = 0;

	if(flagsync && (one_bits >= 0 && one_bits <= 5) && (frame_bit < 8 * 1022)){ // Leave room
	  // not a flag or abort
	  // Previous transition was a zero bit
	  if(zero_pending){
	    // Insert single zero
	    frame_bit++;
	  }
	  // Insert 'one_bits' ones
	  for(int k=0;k<one_bits;k++){
	    hdlc_frame[frame_bit/8] |= 1 << (frame_bit % 8);
	    frame_bit++;
	  }
	  if(one_bits < 5)
	    zero_pending = 1;
	  else
	    zero_pending = 0;  // remove bit-stuffed 0

	} else if(one_bits == 6) {
	  // Flag began with the preceeding zero
	  // Check CRC, process frame
	  if(flagsync){
	    
	    if(frame_bit > 0){
#if 0
	      printf("crc = %x (%s), length = %d bytes %d bits\n",crc,
		     crc == 0xf0b8? "good" : "bad",
		     frame_bit/8,frame_bit%8);
#endif	      
	      int bytes = frame_bit / 8;
              if(crc_good(hdlc_frame,bytes)){
		printf("Packet %d, %d bytes\n",packets++,bytes);
		dump_frame(hdlc_frame,bytes);
	      }
	    }
	  }
	  flagsync = 1;
	  memset(hdlc_frame,0,sizeof(hdlc_frame));
	  frame_bit = 0;
	  zero_pending = 0;
	} else if(one_bits > 6){
	  // Abort
	  flagsync = 0;
	  memset(hdlc_frame,0,sizeof(hdlc_frame));
	  frame_bit = 0;
	  zero_pending = 0;
	}
      } else {
	bits_since_last_edge++;
      }
#endif


      last_val = yval;
    }
    memcpy(old_y,&y[filter->olen - Samppbit],sizeof(old_y));
  }

  return NULL;

}


int main(int argc,char *argv[]){
  setlocale(LC_ALL,getenv("LANG"));

  int c;
  while((c = getopt(argc,argv,"I:v")) != EOF){
    switch(c){
    case 'v':
      Verbose++;
      break;
    case 'I':
      Mcast_address_text = optarg;
      break;
    default:
      fprintf(stderr,"Usage: %s [-v] [-I mcast_address]\n",argv[0]);
      fprintf(stderr,"Defaults: %s -I %s\n",argv[0],Mcast_address_text);
      exit(1);
    }
  }
  


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



