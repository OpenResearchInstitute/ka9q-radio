// $Id$
// Read from HackRF
// Multicast raw 8-bit I/Q samples
// Accept control commands from UDP socket
#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <locale.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <libhackrf/hackrf.h>


#include "sdr.h"
#include "radio.h"
#include "misc.h"
#include "multicast.h"

void *display(void *arg);
pthread_t Display_thread;

// decibel limits for power
float Upper_threshold = -10;
float Lower_threshold = -40;

// samples of all -128 -> +42.14 dB
float DB_offset = 42.14;

int Agc_holdoff = 10;


struct sdrstate {
  // Stuff for sending commands
  struct status status;     // Frequency and gain settings, grouped for transmission in RTP packet

  // Analog gain settings
  int agc_holdoff_count;

  float power;              // Running estimate of signal power - used only by display

  hackrf_device *device;
};

struct sdrstate HackCD;
int ADC_samprate = 1536000;
int Verbose;
int Device;       // Which of several to use

int Blocksize = 700;
int Device = 0;
char *Locale;


int Rtp_sock; // Socket handle for sending real time stream *and* receiving commands
int Ctl_sock;
extern int Mcast_ttl;
long Ssrc;
int Seq = 0;
int Timestamp = 0;

#define BUFFERSIZE (1<<24)
char Sampbuffer[BUFFERSIZE];
int Samp_wp;
int Samp_rp;


pthread_mutex_t Buf_mutex;
pthread_cond_t Buf_cond;


// Callback called with incoming receiver data from A/D
int rx_callback(hackrf_transfer *transfer){
  int remain = transfer->valid_length;
  unsigned char *dp = transfer->buffer;

  while(remain > 0){
    int chunk = min(remain,BUFFERSIZE-Samp_wp);
    memcpy(Sampbuffer + Samp_wp,dp,chunk);
    dp += chunk;
    remain -= chunk;
    pthread_mutex_lock(&Buf_mutex);
    Samp_wp += chunk;
    Samp_wp &= (BUFFERSIZE-1);
    pthread_cond_broadcast(&Buf_cond);
    pthread_mutex_unlock(&Buf_mutex);
  }
  return 0;
}

void *process(void *arg){

  unsigned char buffer[200+2*Blocksize];

  while(1){
    struct rtp_header rtp;
    memset(&rtp,0,sizeof(rtp));
    rtp.version = RTP_VERS;
    rtp.type = IQ_PT8;
    rtp.ssrc = Ssrc;
    rtp.timestamp = Timestamp;
    rtp.seq = Seq++;

    unsigned char *dp = buffer;
    dp = hton_rtp(dp,&rtp);
    dp = hton_status(dp,&HackCD.status);

    pthread_mutex_lock(&Buf_mutex);
    while(1){
      int avail = Samp_wp - Samp_rp;
      if(avail < 0)
	avail += BUFFERSIZE;
      if(avail >= 2*Blocksize)
	break;
      pthread_cond_wait(&Buf_cond,&Buf_mutex);
    }
    pthread_mutex_unlock(&Buf_mutex);
    uint32_t sumsq = 0;
    for(int i=0;i<2*Blocksize;i++){
      int s = Sampbuffer[Samp_rp++]; 
      Samp_rp &= (BUFFERSIZE-1);
      *dp++ = s;
      if(s >= 128)
	s -= 256;
      
      sumsq += s * s;
    }
    HackCD.power += .1 * ((float)sumsq / (2*Blocksize) - HackCD.power);

    if(send(Rtp_sock,buffer,dp - buffer,0) == -1){
      perror("send");
      // If we're sending to a unicast address without a listener, we'll get ECONNREFUSED
      // Sleep 1 sec to slow down the rate of these messages
      usleep(1000000);
    }
    Timestamp += Blocksize; // samples
  
    // Simply increment by number of samples
    // But what if we lose some? Then the clock will always be off
    HackCD.status.timestamp += 1.e9 * (Blocksize) / ADC_samprate;
  }
}


int main(int argc,char *argv[]){
  // if we have root, up our priority and drop privileges
  int prio = getpriority(PRIO_PROCESS,0);
  prio = setpriority(PRIO_PROCESS,0,prio - 10);

  // Quickly drop root if we have it
  // The sooner we do this, the fewer options there are for abuse
  if(seteuid(getuid()) != 0)
    perror("seteuid");

  char *dest = "239.1.6.1"; // Default for testing

  Locale = getenv("LANG");
  if(Locale == NULL || strlen(Locale) == 0)
    Locale = "en_US.UTF-8";

  int c;
  while((c = getopt(argc,argv,"d:vp:l:b:R:T:")) != EOF){
    switch(c){
    case 'R':
      dest = optarg;
      break;
    case 'd':
      Device = strtol(optarg,NULL,0);
      break;
    case 'v':
      Verbose++;
      break;
    case 'l':
      Locale = optarg;
      break;
    case 'b':
      Blocksize = strtol(optarg,NULL,0);
      break;
    case 'T':
      Mcast_ttl = strtol(optarg,NULL,0);
      break;
    }
  }
  if(Verbose)
    fprintf(stderr,"hackrf device %d: blocksize %d\n",Device,Blocksize);

  setlocale(LC_ALL,Locale);
  
  // Set up RTP output socket
  Rtp_sock = setup_mcast(dest,1);
  if(Rtp_sock == -1){
    perror("Can't create multicast socket");
    exit(1);
  }
    
  // Set up control socket
  Ctl_sock = socket(AF_INET,SOCK_DGRAM,0);

  // bind control socket to next sequential port after our multicast source port
  struct sockaddr_in ctl_sockaddr;
  socklen_t siz = sizeof(ctl_sockaddr);
  if(getsockname(Rtp_sock,(struct sockaddr *)&ctl_sockaddr,&siz) == -1){
    perror("getsockname on ctl port");
    exit(1);
  }
  struct sockaddr_in locsock;
  locsock.sin_family = AF_INET;
  locsock.sin_port = htons(ntohs(ctl_sockaddr.sin_port)+1);
  locsock.sin_addr.s_addr = INADDR_ANY;
  bind(Ctl_sock,(struct sockaddr *)&locsock,sizeof(locsock));

  int ret = hackrf_init();
  if(ret != HACKRF_SUCCESS){
    fprintf(stderr,"hackrf_init() failed: %s\n",hackrf_error_name(ret));
    exit(1);
  }
  // Enumerate devices
  hackrf_device_list_t *dlist = hackrf_device_list();

  if((ret = hackrf_device_list_open(dlist,Device,&HackCD.device)) != HACKRF_SUCCESS){
    fprintf(stderr,"hackrf_open(%d) failed: %s\n",Device,hackrf_error_name(ret));
    exit(1);
  }
  hackrf_device_list_free(dlist); dlist = NULL;

  hackrf_set_sample_rate(HackCD.device,(double)ADC_samprate);
  HackCD.status.samprate = ADC_samprate;
  uint32_t bw = hackrf_compute_baseband_filter_bw_round_down_lt(ADC_samprate);
  hackrf_set_baseband_filter_bandwidth(HackCD.device,bw);

  HackCD.status.lna_gain = 40;
  HackCD.status.if_gain = 20;

  hackrf_set_antenna_enable(HackCD.device,0); // Turn off front end amp? 
  hackrf_set_lna_gain(HackCD.device,HackCD.status.lna_gain);
  hackrf_set_vga_gain(HackCD.device,HackCD.status.if_gain);

  pthread_mutex_init(&Buf_mutex,NULL);
  pthread_cond_init(&Buf_cond,NULL);

  pthread_t process_thread;
  pthread_create(&process_thread,NULL,process,NULL);

  hackrf_start_rx(HackCD.device,rx_callback,&HackCD);

  signal(SIGPIPE,SIG_IGN);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        

  if(Verbose > 1)
    pthread_create(&Display_thread,NULL,display,NULL);

  time_t tt;
  time(&tt);

  Ssrc = tt & 0xffffffff; // low 32 bits of clock time

  // Process commands to change hackrf state
  // We listen on the same IP address and port we use as a multicasting source
  pthread_setname("hackrf-cmd");

  while(1){

    fd_set fdset;
    socklen_t addrlen;
    int r;
    struct timeval timeout;
    struct status requested_status;
    
    // Read with a timeout - necessary?
    FD_ZERO(&fdset);
    FD_SET(Ctl_sock,&fdset);
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    r = select(Ctl_sock+1,&fdset,NULL,NULL,&timeout);
    if(r == -1){
      perror("select");
      usleep(50000); // don't loop tightly
      continue;
    }
    if(r == 0)
      continue;

    // A command arrived, read it
    // Should probably log these
    struct sockaddr_in6 command_address;
    addrlen = sizeof(command_address);
    if((r = recvfrom(Ctl_sock,&requested_status,sizeof(requested_status),0,(struct sockaddr *)&command_address,&addrlen)) <= 0){
      if(r < 0)
	perror("recv");
      usleep(50000); // don't loop tightly
      continue;
    }
    
    if(r < sizeof(requested_status))
      continue; // Too short; ignore
    
    uint64_t intfreq = HackCD.status.frequency = requested_status.frequency;
    hackrf_set_freq(HackCD.device,intfreq);
    struct timeval tp;
    gettimeofday(&tp,NULL);
    // Timestamp is in nanoseconds for futureproofing, but time of day is only available in microsec
    HackCD.status.timestamp = ((tp.tv_sec - UNIX_EPOCH + GPS_UTC_OFFSET) * 1000000LL + tp.tv_usec) * 1000LL;
  }
  // Can't really get here
  close(Rtp_sock);
  hackrf_close(HackCD.device);
  hackrf_exit();
  exit(0);
}



// Status display thread
void *display(void *arg){
  pthread_setname("hackrf-disp");
  float powerdB;

  fprintf(stderr,"Frequency      LNA  IF   A/D queue\n");
  fprintf(stderr,"Hz                  dB  dBFS\n");

  while(1){
    powerdB = 10*log10f(HackCD.power) - DB_offset;
    int queue = Samp_wp - Samp_rp;
    if(queue < 0)
      queue += BUFFERSIZE;

    fprintf(stderr,"%'-15.0lf%3d%5d%'6.1f%'10d\r",
	    HackCD.status.frequency,
	    HackCD.status.lna_gain,
	    HackCD.status.if_gain,
	    powerdB,
	    queue);
    usleep(1000000); // 10 Hz
    if(HackCD.agc_holdoff_count){
      HackCD.agc_holdoff_count--;
    } else if(HackCD.power - DB_offset > Upper_threshold){
      // Reduce gain (IF first), start counter
      if(HackCD.status.if_gain > 0){
	HackCD.status.if_gain -= 2;
	hackrf_set_vga_gain(HackCD.device,HackCD.status.if_gain);
	HackCD.agc_holdoff_count = Agc_holdoff;
      } else if(HackCD.status.lna_gain > 0){
	HackCD.status.lna_gain -= 8;
	hackrf_set_lna_gain(HackCD.device,HackCD.status.lna_gain);
	HackCD.agc_holdoff_count = Agc_holdoff;
      }
    } else if(HackCD.power - DB_offset < Lower_threshold){
      // Increase gain (LNA first), start counter
      if(HackCD.status.lna_gain < 40){
	HackCD.status.lna_gain += 8;
	hackrf_set_lna_gain(HackCD.device,HackCD.status.lna_gain);
	HackCD.agc_holdoff_count = Agc_holdoff;
      } else if(HackCD.status.if_gain < 62){
	HackCD.status.if_gain += 2;
	hackrf_set_vga_gain(HackCD.device,HackCD.status.if_gain);
	HackCD.agc_holdoff_count = Agc_holdoff;
      }
    }
  }
  return NULL;
}



// If we don't stop the A/D, it'll take several seconds to overflow and stop by itself,
// and during that time we can't restart
void closedown(int a){
  if(Verbose)
    fprintf(stderr,"hackrf: caught signal %d: %s\n",a,strsignal(a));

  exit(1);
}
