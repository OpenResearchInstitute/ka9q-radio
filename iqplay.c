// $Id$
// Read from IQ recording, multicast in (hopefully) real time
#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <signal.h>
#include <locale.h>
#include <getopt.h>
#include <attr/xattr.h>
#include <sys/time.h>

#include "command.h"
#include "rtp.h"

const int ADC_samprate = 192000;
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
  struct timeval ltv,ntv;
  struct rtp_header rtp;
  FILE *fp;
  struct status status;

  memset(&status,0,sizeof(status));
  int blocksize = 350;
  char *dest = "239.1.2.10"; // Default for testing
  int dest_port = 5004;     // Default for testing; recommended default RTP port
  int dest_is_ipv6 = -1;
  locale = getenv("LANG");

  while((c = getopt(argc,argv,"vl:b:R:P:")) != EOF){
    switch(c){
    case 'R':
      dest = optarg;
      break;
    case 'P':
      dest_port = atoi(optarg);
      break;
    case 'v':
      Verbose++;
      break;
    case 'l':
      locale = optarg;
      break;
    case 'b':
      blocksize = atoi(optarg);
      break;
    }
  }
  if(argc < optind){
    fprintf(stderr,"Usage: %s [options] [filename]\n",argv[0]);
    exit(1);
  }

  setlocale(LC_ALL,locale);
  signal(SIGPIPE,SIG_IGN);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  
  // Set up RTP output socket
  struct sockaddr_in address4;  
  struct sockaddr_in6 address6;
  if(inet_pton(AF_INET,dest,&address4.sin_addr) == 1){
    // Destination is IPv4
    dest_is_ipv6 = 0;
    if((Rtp_sock = socket(PF_INET,SOCK_DGRAM,0)) == -1){
      perror("funcube: can't create IPv4 output socket");
      exit(1);
    }
    address4.sin_family = AF_INET;
    address4.sin_port = htons(dest_port);

    if(IN_MULTICAST(ntohl(address4.sin_addr.s_addr))){
      // Destination is multicast; join it
      struct group_req group_req;
      group_req.gr_interface = 0;
      memcpy(&group_req.gr_group,&address4,sizeof(address4));
      if(setsockopt(Rtp_sock,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0)
	perror("setsockopt ipv4 multicast join group failed");
    } // IN_MULTICAST
  } else if(inet_pton(AF_INET6,dest,&address6.sin6_addr) == 1){
    // Destination is IPv6
    dest_is_ipv6 = 1;
    address6.sin6_family = AF_INET6;
    address6.sin6_flowinfo = 0;  
    address6.sin6_port = htons(dest_port);
    address6.sin6_scope_id = 0;
    if((Rtp_sock = socket(PF_INET6,SOCK_DGRAM,0)) == -1){
      perror("funcube: can't create IPv6 output socket");
      exit(1);
    }
    if(IN6_IS_ADDR_MULTICAST(&address6)){
      // Destination is multicast; join it
      struct group_req group_req;
      group_req.gr_interface = 0;
      memcpy(&group_req.gr_group,&address6,sizeof(address6));
      if(setsockopt(Rtp_sock,IPPROTO_IPV6,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0)
	perror("setsockopt ipv6 multicast join group failed");
    } // IN6_IS_ADDR_MULTICAST
  } else {
    fprintf(stderr,"Can't parse destination %s\n",dest);
    exit(1);
  }
  // Apparently works for both IPv4 and IPv6
  u_char loop = 1;
  if(setsockopt(Rtp_sock,IPPROTO_IP,IP_MULTICAST_LOOP,&loop,sizeof(loop)) != 0)
    perror("setsockopt multicast loop failed");

  u_char ttl = 1;
  if(setsockopt(Rtp_sock,IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof(ttl)) != 0)
    perror("setsockopt multicast ttl failed");
      
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
  if(dest_is_ipv6){
    message.msg_name = &address6;
    message.msg_namelen = sizeof(address6);
  } else if(!dest_is_ipv6){
    message.msg_name = &address4;
    message.msg_namelen = sizeof(address4);
  } else {
    fprintf(stderr,"No valid dest address\n");
    exit(1);
  }
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 3;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  int timestamp = 0;
  int seq = 0;
  int d = 1000000LL * blocksize / ADC_samprate; // microsec between packets
  int i;
  gettimeofday(&ltv,NULL);
  for(i=optind;i<argc;i++){
    if((fp = fopen(argv[i],"r")) == NULL){
      fprintf(stderr,"Can't read %s\n",argv[i]);
      continue;
    }
    char temp[PATH_MAX];
    getxattr(argv[optind],"user.frequency",temp,sizeof(temp));
    status.frequency = atof(temp);
    status.samprate = ADC_samprate;

    while(fread(sampbuf,sizeof(short),2*blocksize,fp) > 0){
      rtp.seq = htons(seq++);
      rtp.timestamp = htonl(timestamp);
      timestamp += blocksize;
      
      // Is it time yet?
      while(1){
	gettimeofday(&ntv,NULL);
	// Microseconds since clock was last updated
	int dt = 1000000 * (ntv.tv_sec - ltv.tv_sec) + ntv.tv_usec - ltv.tv_usec;
	if(dt >= d)
	  break;
	usleep(d-dt);
      }    
      if(sendmsg(Rtp_sock,&message,0) == -1)
	perror("sendmsg");

      // Update to time of next scheduled transmission
      ltv.tv_usec += d;
      if(ltv.tv_usec >= 1000000){
	ltv.tv_usec -= 1000000;
	ltv.tv_sec++;
      }
    }
    fclose(fp);
    fp = NULL;
  }
  close(Rtp_sock);
  exit(0);
}

