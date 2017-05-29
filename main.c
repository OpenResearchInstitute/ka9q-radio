// $Id: main.c,v 1.11 2017/05/29 10:29:04 karn Exp karn $
// Read complex float samples from stdin (e.g., from funcube.c)
// downconvert, filter and demodulate
// Take commands from UDP socket
#define _GNU_SOURCE 1
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#undef I
#include <stdio.h>
#include <stdlib.h>
#include <locale.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "radio.h"
#include "filter.h"
#include "dsp.h"
#include "audio.h"
#include "command.h"
#include "rtp.h"

#define MAXPKT 1500

void closedown(int a);

void *input_loop(void *arg);
int process_command(char *cmdbuf,int len);
pthread_t Display_thread;

int Nthreads = 1;
int ADC_samprate = 192000;
int DAC_samprate = 48000;



int Quiet;
int Ctl_port = 4159;
struct sockaddr_in6 ctl_address;
struct sockaddr_in6 FE_address;

int rtp_sock;
struct sockaddr_in6 rtp_address;
socklen_t rtp_addrlen = sizeof(rtp_address);

int ctl_sock;

int Skips;
int Olds;

int Demod_sock;



int main(int argc,char *argv[]){
  int c,N;
  char *locale;
  char *source = "239.1.2.3"; // Default for testing
  int source_port = 5555;     // Default for testing
  enum mode mode;
  double second_IF;
  struct demod *demod = &Demod;


  locale = getenv("LANG");
  setlocale(LC_ALL,locale);

  fprintf(stderr,"General coverage receiver for the Funcube Pro and Pro+\n");
  fprintf(stderr,"Copyright 2016 by Phil Karn, KA9Q; may be used under the terms of the GNU General Public License\n");
  fprintf(stderr,"Compiled %s on %s\n",__TIME__,__DATE__);

  locale = getenv("LANG");

  // Defaults
  Quiet = 0;
  Audio.name = "sysdefault";
  demod->L = 4096;      // Number of samples in buffer
  demod->M = (4096+1);  // Length of filter impulse response
  mode = FM;
  second_IF = 48000;


  while((c = getopt(argc,argv,"qb:m:l:d:S:L:M:x:h:r:t:c:eT:p:R:P:i:")) != EOF){
    int i;

    switch(c){
    case 'R':
      source = optarg;
      break;
    case 'P':
      source_port = atoi(optarg);
      break;
    case 'p':
      Ctl_port = atoi(optarg);
      break;
    case 'q':
      Quiet++; // Suppress display
      break;
    case 'b':
      Kaiser_beta = atof(optarg);
      break;
    case 'e':
      Audio.echo = 1; // Echo sound to standard output
      break;
    case 'h':
      demod->min_IF = atof(optarg);
      break;
    case 'x':
      demod->max_IF = atof(optarg);
      break;
    case 'L':
      demod->L = atoi(optarg);
      break;
    case 'M':
      demod->M = atoi(optarg);
      break;
    case 'r':
      DAC_samprate = atof(optarg);
      break;
    case 'm':
      for(i = 1; i < Nmodes;i++){
	if(strcasecmp(optarg,Modes[i].name) == 0){
	  mode = Modes[i].mode;
	  break;
	}
      }
      break;
    case 'l':
      locale = optarg;
      break;
    case 'S':
      Audio.name = optarg;
      break;
    case 't':
      Nthreads = atoi(optarg);
      fftwf_init_threads();
      fftwf_plan_with_nthreads(Nthreads);
      fprintf(stderr,"Using %d threads for FFTs\n",Nthreads);
      break;
    case 'i':
      second_IF = atof(optarg);
      break;
    default:
      fprintf(stderr,"Usage: %s [-r DAC sample rate] [-m mode] [-l locale] [-S sound output device] [-L samplepoints] [-M impulsepoints] [-h zeroIFhole] [-x maxIF] [-t threads] [-R multicast_address] [-P port]\n",argv[0]);
      fprintf(stderr,"Default: %s -r %u -m %s -l %s -S %s -L %d -M %d -h %.1f -x %.1f -R %s -P %d\n",
	      argv[0],DAC_samprate,Modes[mode].name,locale,Audio.name,demod->L,demod->M,demod->max_IF,demod->min_IF,
	      source,source_port);
      break;
    }
  }
  if(source == NULL || source_port == -1){
    fprintf(stderr,"Specify -R source_address -P port\n");
    exit(1);
  }
  setlocale(LC_ALL,locale);


  demod->samprate = ADC_samprate;
  Audio.samprate = DAC_samprate;
  // Verify decimation ratio
  if((demod->samprate % Audio.samprate) != 0)
    fprintf(stderr,"Warning: A/D rate %'u is not integer multiple of D/A rate %'u; decimation will probably fail\n",
	    demod->samprate,Audio.samprate);
  demod->decimate = demod->samprate / Audio.samprate;
  
  N = demod->L + demod->M - 1;
  if((N % demod->decimate) != 0)
    fprintf(stderr,"Warning: FFT size %'u is not divisible by decimation ratio %d\n",N,demod->decimate);

  if((demod->M - 1) % demod->decimate != 0)
    fprintf(stderr,"Warning: Filter length %'u - 1 is not divisible by decimation ratio %d\n",demod->M,demod->decimate);

  // Must do this before first filter is created with set_mode(), otherwise a segfault can occur
  fftwf_import_system_wisdom();

  if(demod->max_IF == 0 || demod->max_IF > demod->samprate/2)
    demod->max_IF = demod->samprate/2;
  
  fprintf(stderr,"UDP control port %d\n",Ctl_port);
  fprintf(stderr,"A/D sample rate %'d, D/A sample rate %'d, decimation ratio %'d, max IF +/-%'.1lf Hz\n",
	  demod->samprate,Audio.samprate,demod->decimate,demod->max_IF);
  fprintf(stderr,"block size: %'d complex samples (%'.1f ms @ %'u S/s)\n",
	  demod->L,1000.*demod->L/demod->samprate,demod->samprate);
  fprintf(stderr,"Kaiser beta %'.1lf, impulse response: %'d complex samples (%'.1f ms @ %'u S/s) bin size %'.1f Hz\n",
	  Kaiser_beta,demod->M,1000.*demod->M/demod->samprate,demod->samprate,(float)demod->samprate/N);

  if(!Quiet)
    pthread_create(&Display_thread,NULL,display,NULL);

  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        

  // Set up input socket for multicast data stream from front end
  struct sockaddr_in6 address6;
  struct sockaddr_in address4;  

  if(inet_pton(AF_INET,source,&address4.sin_addr) == 1){
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(source_port);
    address.sin_addr.s_addr = INADDR_ANY;
    if((rtp_sock = socket(PF_INET,SOCK_DGRAM,0)) == -1){
      perror("can't create IPv4 input socket");
      exit(1);
    }
    if(bind(rtp_sock,&address,sizeof(address)) != 0){
      perror("ipv4 bind on input socket failed");
      exit(1);
    }

    struct group_req group_req;
    group_req.gr_interface = 0;
    address4.sin_family = AF_INET;
    address4.sin_port = 0;
    memcpy(&group_req.gr_group,&address4,sizeof(address4));
    if(setsockopt(rtp_sock,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0){
      perror("setsockopt ipv6/v4 multicast join group failed");
      exit(1);
    }
  } else if(inet_pton(AF_INET6,source,&address6.sin6_addr) == 1){
    struct sockaddr_in6 address;
    address.sin6_family = AF_INET6;
    address.sin6_flowinfo = 0;  
    address.sin6_port = htons(source_port);
    address.sin6_scope_id = 0;
    address.sin6_addr = in6addr_any;
    if((rtp_sock = socket(PF_INET6,SOCK_DGRAM,0)) == -1){
      fprintf(stderr,"funcube: can't create IPv6 output socket\n");
      exit(1);
    }
    if(bind(rtp_sock,&address,sizeof(address)) != 0){
      perror("bind to IPv6 multicast address failed");
      exit(1);
    }
    struct group_req group_req;
    group_req.gr_interface = 0;
    address6.sin6_family = AF_INET6;
    address6.sin6_port = htons(source_port);
    memcpy(&group_req.gr_group,&address6,sizeof(address6));
    if(setsockopt(rtp_sock,IPPROTO_IPV6,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0){
      perror("setsockopt ipv6 multicast join group failed");
      exit(1);
    }
  } else {
    fprintf(stderr,"Can't parse RTP source address %s\n",source);
    exit(1);
  }
  u_char loop,ttl;
  loop = 0;
  if(setsockopt(rtp_sock,IPPROTO_IP,IP_MULTICAST_LOOP,&loop,sizeof(loop)) != 0){
    perror("setsockopt multicast loop failed");
  }
  ttl = 1;
  if(setsockopt(rtp_sock,IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof(ttl)) != 0){
    perror("setsockopt multicast ttl failed");
  }

  // Set up control port
  ctl_address.sin6_family = AF_INET6;
  ctl_address.sin6_port = htons(Ctl_port);
  ctl_address.sin6_flowinfo = 0;
  ctl_address.sin6_addr = in6addr_any;
  ctl_address.sin6_scope_id = 0;

  if((ctl_sock = socket(PF_INET6,SOCK_DGRAM, 0)) == -1){
    fprintf(stderr,"can't open control socket\n");
    exit(1);
  }
  if(bind(ctl_sock,&ctl_address,sizeof(ctl_address)) != 0){
    fprintf(stderr,"bind failed\n");
    exit(1);
  }
  fcntl(ctl_sock,F_SETFL,O_NONBLOCK);
  set_mode(&Demod,mode);
  set_second_LO(demod,-second_IF,1);

  int sv[2];
  socketpair(AF_UNIX,SOCK_STREAM,0,sv);
  Demod_sock = sv[0];
  demod->data_sock = sv[1];

  input_loop(NULL);

  exit(0);
}

void closedown(int a){
  if(!Quiet)
    fprintf(stderr,"radio: caught signal %d: %s\n",a,strsignal(a));
  exit(1);
}

// Read from RTP network socket, assemble blocks of samples with corrections done

void *input_loop(void *arg){
  int cnt;
  short samples[MAXPKT];
  struct rtp_header rtp_header;
  struct status status;
  struct iovec iovec[3];
  struct demod *demod = &Demod;

  iovec[0].iov_base = &rtp_header;
  iovec[0].iov_len = sizeof(rtp_header);
  iovec[1].iov_base = &status;
  iovec[1].iov_len = sizeof(status);
  iovec[2].iov_base = samples;
  iovec[2].iov_len = sizeof(samples);
  
  struct msghdr message;
  message.msg_name = &rtp_address;
  message.msg_namelen = sizeof(rtp_address);
  message.msg_iov = iovec;
  message.msg_iovlen = sizeof(iovec) / sizeof(struct iovec);
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  uint16_t seq = 0;

  while(1){
    // See if we have any commands
    socklen_t addrlen;
    int rdlen;
    char pktbuf[MAXPKT];

    while(addrlen = sizeof(ctl_address), (rdlen = recvfrom(ctl_sock,&pktbuf,sizeof(pktbuf),0,&ctl_address,&addrlen)) > 0)
      process_command(pktbuf,rdlen);

    cnt = recvmsg(rtp_sock,&message,0);
    if(cnt <= 0){    // ??
      perror("recvfrom");
      usleep(50000);
      continue;
    }
    if(cnt < sizeof(rtp_header) + sizeof(status))
      continue; // Too small, ignore

    // Look at the RTP header at some point
    rtp_header.seq = ntohs(rtp_header.seq);
    if((int)(seq - rtp_header.seq) < 0){
      Skips++;
    } else if((int)(seq - rtp_header.seq) > 0){
      Olds++;
    }
    seq = rtp_header.seq + 1;

    demod->first_LO = status.frequency;
    demod->lna_gain = status.lna_gain;
    demod->mixer_gain = status.mixer_gain;
    demod->if_gain = status.if_gain;    
    cnt -= sizeof(rtp_header) + sizeof(status);
    cnt /= 4; // count 4-byte stereo samples
    proc_samples(demod,samples,cnt);
  }
}
int process_command(char *cmdbuf,int len){
  struct command command;
  struct demod *demod = &Demod;

  if(len >= sizeof(command)){
    memcpy(&command,cmdbuf,sizeof(command));
    switch(command.cmd){
    case SENDSTAT:
      break; // do this later
    case SETSTATE: // first LO freq, second LO freq, second_LO freq rate, mode
#if 0
      fprintf(stderr,"setstate(%d,%.2lf,%.2lf,%.2lf,%.2lf)\n",
	      command.mode,
	      command.first_LO,
	      command.second_LO,command.second_LO_rate,command.calibrate);
#endif
      // Ignore out-of-range values
      if(command.first_LO > 0 && command.first_LO < 2e9)
	set_first_LO(demod,command.first_LO,0);

      if(command.second_LO >= -demod->samprate/2 && command.second_LO <= demod->samprate/2)
	set_second_LO(demod,command.second_LO,0);
      if(fabs(command.second_LO_rate) < 1e9)
	set_second_LO_rate(demod,command.second_LO_rate,0);
      if(command.mode > 0 && command.mode <= Nmodes)
	set_mode(demod,command.mode);
      if(fabs(command.calibrate) < 1)
	set_cal(demod,command.calibrate);
      break;
    default:
      break; // Ignore
    } // switch
  }
  return 0;
}
 
