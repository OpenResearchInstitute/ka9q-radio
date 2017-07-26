// $Id: iqplay.c,v 1.8 2017/07/23 23:30:21 karn Exp karn $
// Read from IQ recording, multicast in (hopefully) real time
#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <signal.h>
#include <locale.h>
#include <getopt.h>
#include <attr/xattr.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "radio.h"
#include "rtp.h"
#include "dsp.h"
#include "multicast.h"


int ADC_samprate = 192000;
int Verbose;
int Rtp_sock; // Socket handle for sending real time stream

void closedown(int a){
  if(Verbose)
    fprintf(stderr,"iqplay: caught signal %d: %s\n",a,strsignal(a));
  exit(1);
}


int main(int argc,char *argv[]){
  char *locale;
  int c;
  struct timeval start_time;
  struct rtp_header rtp;
  int fd;
  struct status status;
  double frequency = 0;

  memset(&status,0,sizeof(status));
  int blocksize = 256;
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
      blocksize = strtol(optarg,NULL,0);
      break;
    case 'f':
      frequency = strtod(optarg,NULL);
      break;
    }
  }
  if(argc < optind){
    fprintf(stderr,"Usage: %s [options] [filename]\n",argv[0]);
    exit(1);
  }

  setlocale(LC_ALL,locale);
  // Set up RTP output socket
  Rtp_sock = setup_output(dest,dest_port);
      
  long ssrc = 0; // fill this in later

  rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
  rtp.mpt = 97;         // ordinarily dynamically allocated
  rtp.ssrc = htonl(ssrc);

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

  signal(SIGPIPE,SIG_IGN);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  

  int timestamp = 0;
  int seq = 0;
  double dt = (1000000. * blocksize) / ADC_samprate; // microsec between packets
  double sked_time = 0; // Microseconds since start for next scheduled transmission
  gettimeofday(&start_time,NULL);

  int i;
  for(i=optind;i<=argc;i++){
    char *filename;
    if(optind == argc){
      // No arguments, open stdin
      filename = "stdin";
      fd = 0;
    } else {
      filename = argv[i];
      if((fd = open(filename,O_RDONLY)) == -1){
	fprintf(stderr,"Can't read %s\n",filename);
	continue;
      }
    }
    char temp[PATH_MAX+1];
    int n;
    if((n = getxattr(filename,"user.samplerate",temp,sizeof(temp))) > 0){
      temp[n] = '\0';
      status.samprate = strtol(temp,NULL,0);
    } else
      status.samprate = ADC_samprate; // Use default
    if(frequency != 0 || (n = getxattr(filename,"user.frequency",temp,sizeof(temp))) <= 0){
      status.frequency = frequency;
    } else {
      temp[n] = '\0';
      status.frequency = strtod(temp,NULL);
    }
    if(Verbose)
      fprintf(stderr,"Transmitting %s at %'d samp/s upconverted %'.1lf Hz to %s:%s\n",
	      filename,status.samprate,status.frequency,dest,dest_port);

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

      if(sendmsg(Rtp_sock,&message,0) == -1)
	perror("sendmsg");

      // Update time of next scheduled transmission
      sked_time += dt;
    }
    close(fd);
    fd = -1;
  }
  close(Rtp_sock);
  exit(0);
}

