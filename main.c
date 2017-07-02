// $Id: main.c,v 1.42 2017/06/28 04:31:14 karn Exp karn $
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
#include <unistd.h>
#include <fcntl.h>
#include <locale.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>

#include "radio.h"
#include "filter.h"
#include "dsp.h"
#include "audio.h"
#include "command.h"
#include "rtp.h"

#define MAXPKT 1500

void closedown(int a);

void *input_loop(struct demod *);
int process_command(struct demod *,char *cmdbuf,int len);
pthread_t Display_thread;

int Nthreads = 1;
int ADC_samprate = 192000;
int DAC_samprate = 48000;

int Quiet;
int Verbose;
int Ctl_port = 4159;

struct sockaddr_in Ctl_address;
struct sockaddr_in Input_source_address;

char *IQ_mcast_address_text;
int Mcast_dest_port;
char *BB_mcast_address_text;
struct sockaddr_in BB_mcast_sockaddr;
int Send_OPUS; // send OPUS audio if 1, PCM if 0

int Input_fd;
int Ctl_fd;
int Demod_sock;


int setup_input(char *addr){
  int fd = -1;
  struct sockaddr_in sock;

  // Set up input socket for multicast data stream from front end
  if(inet_pton(AF_INET,addr,&sock.sin_addr) != 1)
    return -1;

  if((fd = socket(PF_INET,SOCK_DGRAM,0)) == -1){
    perror("can't create IPv4 input socket");
    return -1;
  }

  int reuse = 1;
  if(setsockopt(fd,SOL_SOCKET,SO_REUSEPORT,&reuse,sizeof(reuse)) != 0)
    perror("ipv4 so_reuseport failed");
  if(setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse)) != 0)
    perror("ipv4 so_reuseaddr failed");
  
  sock.sin_family = AF_INET;
  sock.sin_port = htons(Mcast_dest_port);
  if(bind(fd,(struct sockaddr *)&sock,sizeof(sock)) != 0){
    perror("bind on IPv4 input socket");
    return -1;
  }
  
#if 1 // old version, seems required on Apple    
  struct ip_mreq mreq;
  mreq.imr_multiaddr = sock.sin_addr;
  mreq.imr_interface.s_addr = INADDR_ANY;
  if(setsockopt(fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq)) != 0)
    perror("ipv4 multicast join");
  
#else // Linux, etc
  struct group_req group_req;
  group_req.gr_interface = 0;
  sock.sin_family.sin_family = AF_INET;
  memcpy(&group_req.gr_group,&sock,sizeof(sock));
  if(setsockopt(fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0)
    perror("ipv4 multicast join");
  
#endif
  return fd;
}

int Skips;
int Delayed;

int main(int argc,char *argv[]){
  int c,N;
  char *locale;

  enum mode mode;
  double second_IF;
  struct demod *demod = &Demod;

  locale = getenv("LANG");
  setlocale(LC_ALL,locale);


  // Defaults
  Quiet = 0;
  // The FFT length will be L + M - 1 because of window overlapping
  demod->L = 4096;      // Number of samples in buffer
  demod->M = 4096+1;    // Length of filter impulse response
  mode = FM;
  second_IF = ADC_samprate/4;

  IQ_mcast_address_text = strdup("239.1.2.1");
  BB_mcast_address_text = strdup("239.2.1.1");
  OPUS_bitrate = 32000;
  Mcast_dest_port = 5004;     // recommended default RTP port
  Send_OPUS = 0;
  OPUS_blocktime = 20;

  while((c = getopt(argc,argv,"B:c:i:I:k:l:L:m:M:p:R:qr:t:v")) != EOF){
    int i;

    switch(c){
    case 'B':
      OPUS_blocktime = strtod(optarg,NULL);
      break;
    case 'c':
      demod->calibrate = 1e-6 * strtod(optarg,NULL);
      break;
    case 'i':
      second_IF = strtod(optarg,NULL);
      break;
    case 'I':
      if(IQ_mcast_address_text)
	free(IQ_mcast_address_text);
      IQ_mcast_address_text = strdup(optarg);
      break;
    case 'k':
      Kaiser_beta = strtod(optarg,NULL);
      break;
    case 'l':
      locale = optarg;
      break;
    case 'L':
      demod->L = strtol(optarg,NULL,0);
      break;
    case 'm':
      for(i = 0; i < Nmodes;i++){
	if(strcasecmp(optarg,Modes[i].name) == 0){
	  mode = Modes[i].mode;
	  break;
	}
      }
      break;
    case 'M':
      demod->M = strtol(optarg,NULL,0);
      break;
      break;
    case 'p':
      Ctl_port = strtol(optarg,NULL,0);
      break;
    case 'q':
      Quiet++; // Suppress display
      break;
    case 'r':
      OPUS_bitrate = strtol(optarg,NULL,0);
      break;
    case 'R':
      if(BB_mcast_address_text)
	free(BB_mcast_address_text);
      BB_mcast_address_text = strdup(optarg);
      break;
    case 't':
      Nthreads = strtol(optarg,NULL,0);
      fftwf_init_threads();
      fftwf_plan_with_nthreads(Nthreads);
      fprintf(stderr,"Using %d threads for FFTs\n",Nthreads);
      break;
    case 'v':
      Verbose++;
      break;
    default:
      fprintf(stderr,"Usage: %s [-B opus_blocktime] [-c calibrate_ppm] [-I iq multicast address] [-l locale] [-L samplepoints] [-m mode] [-M impulsepoints] [-O Opus multicast address] [-P PCM multicast address] [-r opus_bitrate] [-t threads]\n",argv[0]);
      fprintf(stderr,"Default: %s -B %.1f -c %.2lf -I %s -l %s -L %d -m %s -M %d -R %s -r %d -t %d\n",
	      argv[0],OPUS_blocktime,demod->calibrate*1e6,IQ_mcast_address_text,locale,demod->L,Modes[mode].name,demod->M,
	      BB_mcast_address_text,OPUS_bitrate,Nthreads);
      exit(1);
      break;
    }
  }
  if(IQ_mcast_address_text == NULL){
    fprintf(stderr,"Specify -I iq_mcast_address_text_address\n");
    exit(1);
  }
  if(OPUS_bitrate == 0)
    Send_OPUS = 0; // Force PCM

  setlocale(LC_ALL,locale);

  demod->samprate = ADC_samprate * (1 + demod->calibrate);
  demod->min_IF = -80000; // Hardwired for Funcube
  demod->max_IF = +80000;
  demod->decimate = ADC_samprate / DAC_samprate;
  // Verify decimation ratio
  if((ADC_samprate % DAC_samprate) != 0)
    fprintf(stderr,"Warning: A/D rate %'u is not integer multiple of D/A rate %'u; decimation will probably fail\n",
	    ADC_samprate,DAC_samprate);
  
  N = demod->L + demod->M - 1;
  if((N % demod->decimate) != 0)
    fprintf(stderr,"Warning: FFT size %'u is not divisible by decimation ratio %d\n",N,demod->decimate);

  if((demod->M - 1) % demod->decimate != 0)
    fprintf(stderr,"Warning: Filter length %'u - 1 is not divisible by decimation ratio %d\n",demod->M,demod->decimate);

  // Must do this before first filter is created with set_mode(), otherwise a segfault can occur
  fftwf_import_system_wisdom();

  if(Verbose){
    fprintf(stderr,"General coverage receiver for the Funcube Pro and Pro+\n");
    fprintf(stderr,"Copyright 2016 by Phil Karn, KA9Q; may be used under the terms of the GNU General Public License\n");
    fprintf(stderr,"Compiled %s on %s\n",__TIME__,__DATE__);
    fprintf(stderr,"Nmodes = %d\n",Nmodes);
    fprintf(stderr,"UDP control port %d\n",Ctl_port);
    fprintf(stderr,"A/D sample rate %'d, D/A sample rate %'d, decimation ratio %'d\n",
	  ADC_samprate,DAC_samprate,demod->decimate);
    fprintf(stderr,"block size: %'d complex samples (%'.1f ms @ %'u S/s)\n",
	  demod->L,1000.*demod->L/ADC_samprate,ADC_samprate);
    fprintf(stderr,"Kaiser beta %'.1lf, impulse response: %'d complex samples (%'.1f ms @ %'u S/s) bin size %'.1f Hz\n",
	    Kaiser_beta,demod->M,1000.*demod->M/ADC_samprate,ADC_samprate,(float)ADC_samprate/N);
  }
  if(!Quiet)
    pthread_create(&Display_thread,NULL,display,NULL);

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  signal(SIGPIPE,SIG_IGN);

  Input_fd = setup_input(IQ_mcast_address_text);

  {
  // Set up control port
    struct sockaddr_in sock;
    sock.sin_family = AF_INET;
    sock.sin_port = htons(Ctl_port);
    sock.sin_addr.s_addr = INADDR_ANY;

    if((Ctl_fd = socket(PF_INET,SOCK_DGRAM, 0)) == -1)
      perror("can't open control socket");

    if(bind(Ctl_fd,&sock,sizeof(struct sockaddr_in)) != 0)
      perror("control bind failed");
  }
  // Set up audio output stream(s)
  Mcast_fd = socket(AF_INET,SOCK_DGRAM,0);

  // I've given up on IPv6 multicast for now. Too many bugs in too many places
  if(BB_mcast_address_text != NULL && strlen(BB_mcast_address_text) > 0){
    BB_mcast_sockaddr.sin_family = AF_INET;
    BB_mcast_sockaddr.sin_port = htons(Mcast_dest_port);
    inet_pton(AF_INET,BB_mcast_address_text,&BB_mcast_sockaddr.sin_addr);

    // Strictly speaking, it is not necessary to join a multicast group to which we only send.
    // But this creates a problem with brain-dead Netgear (and probably other) "smart" switches
    // that do IGMP snooping. There's a setting to handle what happens with multicast groups
    // to which no IGMP messages are seen. If set to discard them, IPv6 multicast breaks
    // because there's no IPv6 multicast querier. But set to pass them, then IPv4 multicasts
    // that aren't subscribed to by anybody are flooded everywhere! We avoid that by subscribing
    // to our own multicasts.
#if __APPLE__ // Newer, protocol-independent MCAST_JOIN_GROUP doesn't seem to work on OSX
    struct ip_mreq mreq;
    mreq.imr_multiaddr = BB_mcast_sockaddr.sin_addr;
    mreq.imr_interface.s_addr = INADDR_ANY;
    if(setsockopt(Mcast_fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq)) != 0)
      perror("ipv4 multicast join");

#else // Linux, etc
    struct group_req group_req;
    group_req.gr_interface = 0;
    BB_mcast_sockaddr.sin_family = AF_INET;
    memcpy(&group_req.gr_group,&BB_mcast_sockaddr,sizeof(struct sockaddr_in));
    if(setsockopt(Mcast_fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0)
      perror("ipv4 multicast join");
#endif
  }
  // Apparently works for both IPv4 and IPv6
  int ttl = 2;
  if(setsockopt(Mcast_fd,IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof(ttl)) != 0)
    perror("setsockopt multicast ttl failed");

  // Pipes for front end -> demod
  int sv[2];
  pipe(sv);
  Demod_sock = sv[1]; // write end
  demod->input = sv[0]; // read end
#ifdef F_SETPIPE_SZ // Linux only
  int sz;
  sz = fcntl(Demod_sock,F_SETPIPE_SZ,demod->L * sizeof(complex float));
#if 0
  fprintf(stderr,"sock size %d\n",sz);
#endif
  if(sz == -1)
    perror("F_SETPIPE_SZ");
#endif // F_SETPIPE_SZ
  if(setup_audio() != 0){
    fprintf(stderr,"Audio setup failed\n");
    exit(1);
  }

  set_mode(demod,mode);
  set_second_LO(demod,-second_IF);
  input_loop(demod); // Doesn't return

  exit(0);
}

void display_cleanup(void *);

void closedown(int a){
  if(!Quiet)
    fprintf(stderr,"radio: caught signal %d: %s\n",a,strsignal(a));
  display_cleanup(NULL);

  exit(1);
}

// Read from RTP network socket, assemble blocks of samples with corrections done

void *input_loop(struct demod *demod){
  int16_t samples[MAXPKT];
  struct rtp_header rtp;
  struct status status;
  struct iovec iovec[3];

  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = &status;
  iovec[1].iov_len = sizeof(status);
  iovec[2].iov_base = samples;
  iovec[2].iov_len = sizeof(samples);
  
  struct msghdr message;
  message.msg_name = &Input_source_address;
  message.msg_namelen = sizeof(Input_source_address);
  message.msg_iov = iovec;
  message.msg_iovlen = sizeof(iovec) / sizeof(struct iovec);
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  int eseq = -1;

  while(1){
    fd_set mask;
    FD_ZERO(&mask);
    FD_SET(Ctl_fd,&mask);
    FD_SET(Input_fd,&mask);
    
    select(max(Ctl_fd,Input_fd)+1,&mask,NULL,NULL,NULL);

    if(FD_ISSET(Ctl_fd,&mask)){
      // Got a command
      socklen_t addrlen;
      addrlen = sizeof(Ctl_address);
      char pktbuf[MAXPKT];
      int rdlen;
      rdlen = recvfrom(Ctl_fd,&pktbuf,sizeof(pktbuf),0,(struct sockaddr *)&Ctl_address,&addrlen);
      // Should probably look at the source address
      if(rdlen > 0)
	process_command(demod,pktbuf,rdlen);
    }
    if(FD_ISSET(Input_fd,&mask)){
      // Receive I/Q data from front end
      int cnt;
      cnt = recvmsg(Input_fd,&message,0);
      if(cnt <= 0){    // ??
	perror("recvfrom");
	usleep(50000);
	continue;
      }
      if(cnt < sizeof(rtp) + sizeof(status))
	continue; // Too small, ignore
      
      // Host byte order
      rtp.ssrc = ntohl(rtp.ssrc);
      rtp.seq = ntohs(rtp.seq);
      rtp.timestamp = ntohl(rtp.timestamp);
      if(eseq != -1 && (int16_t)(eseq - rtp.seq) < 0){
	Skips++;
      } else if(eseq != -1 && (int16_t)(eseq - rtp.seq) > 0){
	Delayed++;
      }
      eseq = rtp.seq + 1;
      
      demod->first_LO = status.frequency;
      demod->lna_gain = status.lna_gain;
      demod->mixer_gain = status.mixer_gain;
      demod->if_gain = status.if_gain;    
      cnt -= sizeof(rtp) + sizeof(status);
      cnt /= 4; // count 4-byte stereo samples
      proc_samples(demod,samples,cnt);
    }
  }
}
int process_command(struct demod *demod,char *cmdbuf,int len){
  struct command command;

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
	set_second_LO(demod,command.second_LO);
      if(fabs(command.second_LO_rate) < 1e9)
	set_second_LO_rate(demod,command.second_LO_rate,0);
      if(command.mode >= 0 && command.mode < Nmodes)
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
 
