// $Id: iqplay.c,v 1.11 2017/07/29 21:42:33 karn Exp karn $
// Read from IQ recording, multicast in (hopefully) real time
#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <signal.h>
#include <locale.h>
#include <getopt.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "radio.h"
#include "rtp.h"
#include "dsp.h"
#include "multicast.h"
#include "attr.h"


int Verbose;
int Rtp_sock; // Socket handle for sending real time stream
double Default_frequency = 0;
long Default_samprate = 192000;
int Blocksize = 256;

void closedown(int a){
  if(Verbose)
    fprintf(stderr,"iqplay: caught signal %d: %s\n",a,strsignal(a));
  exit(1);
}


// Play I/Q file with descriptor 'fd' on network socket 'sock'
int playfile(int sock,int fd,int blocksize){
  struct status status;
  memset(&status,0,sizeof(status));
  status.samprate = Default_samprate; // Not sure this is useful
  status.frequency = Default_frequency;
  attrscanf(fd,"samplerate","%ld",&status.samprate);
  attrscanf(fd,"frequency","%lf",&status.frequency);


  if(Verbose)
    fprintf(stderr," %'d samp/s, RF LO %'.1lf Hz\n",status.samprate,status.frequency);

  struct rtp_header rtp;
  rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
  rtp.mpt = 97;         // ordinarily dynamically allocated

  
  struct timeval start_time;
  gettimeofday(&start_time,NULL);
  rtp.ssrc = htonl(start_time.tv_sec);
  int timestamp = 0;
  int seq = 0;
  
  // microsec between packets. Double precision is used to avoid small errors that could
  // accumulate over time
  double dt = (1000000. * blocksize) / status.samprate;
  // Microseconds since start for next scheduled transmission; will transmit first immediately
  double sked_time = 0;
  
  short sampbuf[2*blocksize];
  struct iovec iovec[3];
  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = &status;
  iovec[1].iov_len = sizeof(status);
  iovec[2].iov_base = sampbuf;
  iovec[2].iov_len = sizeof(sampbuf);

  struct msghdr message;
  message.msg_name = NULL;
  message.msg_namelen = 0;
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 3;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  while(fillbuf(fd,sampbuf,sizeof(sampbuf)) > 0){
    rtp.seq = htons(seq++);
    rtp.timestamp = htonl(timestamp);
    timestamp += blocksize;
    
    // Is it time yet?
    while(1){
      // Microseconds since start
      struct timeval tv,diff;
      gettimeofday(&tv,NULL);
      timersub(&tv,&start_time,&diff);
      double rt = 1000000. * diff.tv_sec + diff.tv_usec;
      if(rt >= sked_time)
	break;
      if(sked_time > rt + 100){
	// Use care here, s is unsigned
	useconds_t s = (sked_time - rt) - 100; // sleep until 100 microseconds before
	usleep(s);
      }
    }
    if(sendmsg(sock,&message,0) == -1)
      perror("sendmsg");
    
    // Update time of next scheduled transmission
    sked_time += dt;
  }
  return 0;
}



int main(int argc,char *argv[]){
  char *locale;
  int c;



  char *dest = "239.1.2.10"; // Default for testing
  char *dest_port = "5004";     // Default for testing; recommended default RTP port
  locale = getenv("LANG");

  while((c = getopt(argc,argv,"vl:b:R:P:f:")) != EOF){
    switch(c){
    case 'R':
      dest = optarg;
      break;
    case 'P':
      dest_port = optarg;
      break;
    case 'v':
      Verbose++;
      break;
    case 'l':
      locale = optarg;
      break;
    case 'b':
      Blocksize = strtol(optarg,NULL,0);
      break;
    case 'f': // Used only if there's no tag on a file, or for stdin
      Default_frequency = strtod(optarg,NULL);
      break;
    }
  }
  if(argc < optind){
    fprintf(stderr,"Usage: %s [options] [filename]\n",argv[0]);
    exit(1);
  }

  setlocale(LC_ALL,locale);
  // Set up RTP output socket
  Rtp_sock = setup_mcast(dest,dest_port,1);

  signal(SIGPIPE,SIG_IGN);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  

  if(optind == argc){
    // No file arguments, read from stdin
    if(Verbose)
      fprintf(stderr,"Transmitting from stdin ");
    playfile(Rtp_sock,0,Blocksize);
  } else {
    int i;
    for(i=optind;i<argc;i++){
      int fd;
      if((fd = open(argv[i],O_RDONLY)) == -1){
	fprintf(stderr,"Can't read %s; ",argv[i]);
	perror("");
	continue;
      }
      if(Verbose)
	fprintf(stderr,"Transmitting %s ",argv[i]);
      playfile(Rtp_sock,fd,Blocksize);
      close(fd);
      fd = -1;
    }
  }
  close(Rtp_sock);
  Rtp_sock = -1;
  exit(0);
}

