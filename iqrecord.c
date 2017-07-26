// $Id: iqrecord.c,v 1.2 2017/07/23 23:30:44 karn Exp karn $
// Read complex float samples from stdin (e.g., from funcube.c)
// write into file
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
#include <sys/types.h>
#include <attr/xattr.h>

#include "radio.h"
#include "rtp.h"

#define NSESSIONS 20

struct session {
  FILE *fp;   // File being recorded
  uint32_t ssrc;
  uint32_t timestamp; // first timestamp
  uint16_t seq; // first sequence number
  time_t lastused;
  double frequency;
  struct sockaddr_in iq_sender;
  unsigned int samprate;
} Sessions[NSESSIONS];

#define MAXPKT 1500

void closedown(int a);

void *input_loop(void);

int ADC_samprate = 192000;

int Quiet;

struct sockaddr_in sender;
struct sockaddr_in Input_mcast_sockaddr;

int Input_fd;
char Mcast_dest_port[] = "5004";     // Default for testing; recommended default RTP port

int main(int argc,char *argv[]){
  int c;
  char *locale;

  locale = getenv("LANG");
  setlocale(LC_ALL,locale);

  fprintf(stderr,"I/Q raw signal recorder for the Funcube Pro and Pro+\n");
  fprintf(stderr,"Copyright 2016 by Phil Karn, KA9Q; may be used under the terms of the GNU General Public License\n");
  fprintf(stderr,"Compiled %s on %s\n",__TIME__,__DATE__);

  // Defaults
  Quiet = 0;
  char *iq_mcast_address_text = "239.1.2.3"; // Default for testing

  while((c = getopt(argc,argv,"I:l:q")) != EOF){
    switch(c){
    case 'I':
      iq_mcast_address_text = optarg;
      break;
    case 'l':
      locale = optarg;
      break;
    case 'q':
      Quiet++; // Suppress display
      break;
    default:
      fprintf(stderr,"Usage: %s [-I iq multicast address] [-l locale] [-q]\n",argv[0]);
      fprintf(stderr,"Default: %s -I %s -l %s\n",
	      argv[0],iq_mcast_address_text,locale);
      exit(1);
      break;
    }
  }
  if(iq_mcast_address_text == NULL){
    fprintf(stderr,"Specify -I iq_mcast_address_text_address\n");
    exit(1);
  }
  setlocale(LC_ALL,locale);

  fprintf(stderr,"A/D sample rate %'d\n",ADC_samprate);

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  signal(SIGPIPE,SIG_IGN);

  // Set up input socket for multicast data stream from front end
  Input_fd = setup_input(iq_mcast_address_text,Mcast_dest_port);
  if(Input_fd == -1){
    fprintf(stderr,"Can't set up I/Q input\n");
    exit(1);
  }
  input_loop(); // Doesn't return

  exit(0);
}

void closedown(int a){
  if(!Quiet)
    fprintf(stderr,"radio: caught signal %d: %s\n",a,strsignal(a));

  exit(1);
}

// Read from RTP network socket, assemble blocks of samples with corrections done

void *input_loop(){
  int cnt;
  short samples[MAXPKT];
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
  message.msg_name = &sender;
  message.msg_namelen = sizeof(sender);
  message.msg_iov = iovec;
  message.msg_iovlen = sizeof(iovec) / sizeof(struct iovec);
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  while(1){

    // Receive I/Q data from front end
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
    
    struct session *sp = NULL;
    int i;

    for(i=0;i<NSESSIONS;i++){
      if(Sessions[i].ssrc == rtp.ssrc
	 && memcmp(&Sessions[i].iq_sender,&sender,sizeof(struct sockaddr_in)) == 0
	 && Sessions[i].frequency == status.frequency){
	sp = &Sessions[i];
	break;
      }
    }
    if(sp == NULL){ // Not found; look for empty slot
      time_t tt = time(NULL);
      for(i=0;i<NSESSIONS;i++){
	if(Sessions[i].fp == NULL || Sessions[i].lastused + 30 < tt){
	  sp = &Sessions[i];
	  if(sp->fp != NULL){
	    fprintf(stderr,"Closing old session %d\n",i);
	    fclose(sp->fp);
	    sp->fp = NULL;
	  }
	  // Initialize entry
	  char filename[PATH_MAX];
	  snprintf(filename,sizeof(filename),"iqrecord-%.1lfHz-%lx",status.frequency,(long unsigned)rtp.ssrc);
	  fprintf(stderr,"create session %ld -> %s\n",(long)(sp - Sessions),filename);
	  memcpy(&sp->iq_sender,&sender,sizeof(struct sockaddr_in));
	  sp->frequency = status.frequency;
	  sp->fp = fopen(filename,"w+");
	  char temp[PATH_MAX];
	  snprintf(temp,sizeof(temp),"%lu",(unsigned long)status.samprate);
	  setxattr(filename,"user.samplerate",temp,strlen(temp),0);
	  snprintf(temp,sizeof(temp),"%.1f",status.frequency);
	  setxattr(filename,"user.frequency",temp,strlen(temp),0);
	  snprintf(temp,sizeof(temp),"%lx",(long unsigned)rtp.ssrc);
	  setxattr(filename,"user.ssrc",temp,strlen(temp),0);

	  char temp2[PATH_MAX];
	  inet_ntop(AF_INET,&sender.sin_addr,temp2,sizeof(temp2));
	  int sport;
	  sport = ntohs(sender.sin_port);
	  snprintf(temp,sizeof(temp),"%s:%d",temp2,sport);
	  setxattr(filename,"user.source",temp,strlen(temp),0);

	  sp->ssrc = rtp.ssrc;
	  sp->seq = rtp.seq;
	  sp->timestamp = rtp.timestamp;
	  break;
	}
      }
    }
    if(sp == NULL){
      char src[INET6_ADDRSTRLEN];
      int sport = -1;
      if(sender.sin_family == AF_INET){
	inet_ntop(AF_INET,&((struct sockaddr_in *)&sender)->sin_addr,src,sizeof(src));      
	sport = ntohs(((struct sockaddr_in *)&sender)->sin_port);
      } else if(sender.sin_family == AF_INET6){
	inet_ntop(AF_INET6,&((struct sockaddr_in6 *)&sender)->sin6_addr,src,sizeof(src));
	sport = ntohs(((struct sockaddr_in6 *)&sender)->sin6_port);
      } else {
	strcpy(src,"unknown");
      }
      fprintf(stderr,"No slots available for ssrc 0x%x from %s:%d\n",(unsigned int)rtp.ssrc,src,sport);
      continue; // Drop packet
    }
    sp->lastused = time(NULL);

    fseek(sp->fp,2 * sizeof(int16_t) * (rtp.timestamp - sp->timestamp),SEEK_SET);

    cnt -= sizeof(rtp) + sizeof(status);
    fwrite(samples,sizeof(samples[0]),cnt/2,sp->fp);
  }
}
 
