// $Id$
// Process AX.25 frames containing APRS data, feed to APRS2 network
// Copyright 2018, Phil Karn, KA9Q

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
#include <math.h>
#include <time.h>
#include <fcntl.h>

#include "multicast.h"
#include "ax25.h"
#include "misc.h"

char *Mcast_address_text = "ax25.vhf.mcast.local:8192";
char *Host = "noam.aprs2.net";
char *Port = "14580";

int Verbose;
int Input_fd = -1;
int All;
int Network_fd = -1;

int monstring(char *output,int outlen, unsigned char *packet,int pktlen);

int main(int argc,char *argv[]){
  setlocale(LC_ALL,getenv("LANG"));

  int c;
  while((c = getopt(argc,argv,"I:vah:p:")) != EOF){
    switch(c){
    case 'a':
      All++;
      break;
    case 'v':
      Verbose++;
      break;
    case 'h':
      Host = optarg;
      break;
    case 'p':
      Port = optarg;
      break;
    case 'I':
      Mcast_address_text = optarg;
      break;
    default:
      fprintf(stderr,"Usage: %s [-v] [-I mcast_address][-h host][-p port]\n",argv[0]);
      fprintf(stderr,"Defaults: %s -I %s\n",argv[0],Mcast_address_text);
      exit(1);
    }
  }
  printf("APRS feeder program by KA9Q\n");
  struct addrinfo hints;
  memset(&hints,0,sizeof(hints));
  hints.ai_family = AF_INET; // Only IPv4 for now (grrr....)
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_NUMERICSERV;

  struct addrinfo *results = NULL;
  int ecode;
  if((ecode = getaddrinfo(Host,Port,&hints,&results)) != 0){
    fprintf(stderr,"Can't getaddrinfo(%s,%s): %s\n",Host,Port,gai_strerror(ecode));
    exit(1);
  }
  if((Network_fd = socket(AF_INET,SOCK_STREAM,0)) < 0){
    perror("socket");
    exit(1);
  }
  struct addrinfo *resp;
  for(resp = results; resp != NULL; resp = resp->ai_next){
    if(connect(Network_fd,resp->ai_addr,resp->ai_addrlen) == 0)
      break;
  }
  if(resp == NULL){
    fprintf(stderr,"Unsuccessful connection to server\n");
    exit(1);
  }
  freeaddrinfo(results);
  // Make non-blocking so we can just poll once in a while
  if(fcntl(Network_fd,F_SETFL,O_NONBLOCK) < 0)
    perror("fcntl");


  FILE *Net_fp = fdopen(Network_fd,"w+");
  fprintf(Net_fp,"user KA9Q-1 pass 498 vers KA9Q-aprs 1.0\r\n");
  //  fprintf(Net_fp,"user W6SUN-1 pass 14721 vers KA9Q-aprs 1.0\r\n");
  fflush(Net_fp);

  // Set up multicast input
  Input_fd = setup_mcast(Mcast_address_text,0);
  if(Input_fd == -1){
    fprintf(stderr,"Can't set up input from %s\n",
	    Mcast_address_text);
    exit(1);
  }
  unsigned char packet[2048];
  int pktlen;

  while((pktlen = recv(Input_fd,packet,sizeof(packet),0)) > 0){
    time_t t;
    struct tm *tmp;
    time(&t);
    tmp = gmtime(&t);
    printf("%d %s %04d %02d:%02d:%02d UTC: ",tmp->tm_mday,Months[tmp->tm_mon],tmp->tm_year+1900,
	   tmp->tm_hour,tmp->tm_min,tmp->tm_sec);
		
    char outstring[1024];
    // pktlen includes the CRC, don't pass it to monstring
    if(monstring(outstring,sizeof(outstring),packet,pktlen-2) < 0){
      printf("error in monstring\n");
      continue;
    }
    // Find end of address field to look for type and protocol
    int i;
    for(i=0;i<pktlen-2 && !(packet[i] & 1);i++)
      ;
    if(i >= pktlen-2){
      printf("Can't find control field!\n");
      continue;
    }
    if(packet[i+1] != 0x03 || packet[i+2] != 0xf0){
      printf("Invalid ax25 ctl/protocol\n");
      continue;
    }
    printf("%s\n",outstring);
    // Poll the network socket for input (kludge - should be separate thread)
    fprintf(Net_fp,"%s\r\n",outstring);
    fflush(Net_fp);
    
    char c;
    while(read(Network_fd,&c,1) == 1){
      putchar(c);
    }

  }
}

// construct formatted string in TNC2 monitor format
int monstring(char *output,int outlen, unsigned char *packet,int pktlen){
  // Ensure there's an end to the address field
  if(pktlen < 16) // 7+7+1+1 = 16
    return -1; // Too short

  int i;
  for(i = 14; i < pktlen && !(packet[i] & 1);i += 7)
    ;

  if(i >= pktlen){
    // Invalid!
    return -1;
  }

  int totalwritten = 0;
  char source[10],dest[10];
  get_callsign(source,packet+7);
  get_callsign(dest,packet+0);

  int written = snprintf(output,outlen,"%s>%s",source,dest);
  if(written < 0)
    return written;
  output += written; outlen -= written;
  if(outlen <= 0)
    return -1;
  totalwritten += written;

  for(i=14;i+6 < pktlen && !(packet[i-1] & 1);i += 7){
    char digi[10];
    get_callsign(digi,packet+i);
    written = snprintf(output,outlen,",%s%s",digi,(packet[i+6] & 0x80) ? "*":"");
    if(written < 0)
      return written;
    output += written; outlen -= written;    
    if(outlen <= 0)
      return -1;
    totalwritten += written;
  }
  i += 2; // Advance over type and protocol
  *output++ = ':';
  outlen--;
  if(outlen <= 0)
    return -1;
  totalwritten++;
  for(;i<pktlen;i++){
    if(packet[i] == '\r' || packet[i] == '\n')
      break;
    *output++ = packet[i] & 0x7f; // Strip parity
    outlen--;
    totalwritten++;
    if(outlen <= 0)
      return -1;
  }
  *output++ = '\0';
  return totalwritten;
}
