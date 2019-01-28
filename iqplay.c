// $Id: iqplay.c,v 1.30 2018/12/02 09:16:45 karn Exp karn $
// Read from IQ recording, multicast in (hopefully) real time
// Copyright 2018 Phil Karn, KA9Q
#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <signal.h>
#include <locale.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <fcntl.h>
#include <getopt.h>

#include "misc.h"
#include "radio.h"
#include "multicast.h"
#include "attr.h"
#include "status.h"
#include "dsp.h"


int Verbose;
int Mcast_ttl = 1; // Don't send fast IQ streams beyond the local network by default
double Default_frequency = 0;
long Samprate = 192000;
const int Bufsize = 16384;
int Blocksize = 256;
char *Description;
struct sockaddr_storage Output_data_dest_address;
struct sockaddr_storage Output_data_source_address;
struct sockaddr_storage Output_metadata_dest_address;
struct sockaddr_storage Output_metadata_source_address;
uint64_t Commands;
struct state State[256];
uint64_t Output_metadata_packets;
uint32_t Ssrc;
float Power;
struct rtp_state Rtp_state;

int Status_sock = -1;
int Rtp_sock = -1; // Socket handle for sending real time stream
int Nctl_sock = -1;



void send_iqplay_status(int full);

// Play I/Q file with descriptor 'fd' on network socket 'sock'
int playfile(int sock,int fd,int blocksize){
  struct status status;
  memset(&status,0,sizeof(status));
  status.samprate = Samprate; // Not sure this is useful
  status.frequency = Default_frequency;
  attrscanf(fd,"samplerate","%ld",&status.samprate);
  attrscanf(fd,"frequency","%lf",&status.frequency);
  if(attrscanf(fd,"source_timestamp","%lld",&status.timestamp) == -1){
    double unixstarttime;
    attrscanf(fd,"unixstarttime","%lf",&unixstarttime);
    // Convert decimal seconds from UNIX epoch to integer nanoseconds from GPS epoch
    status.timestamp = (unixstarttime  - UNIX_EPOCH + GPS_UTC_OFFSET) * 1000000000LL;
  }
  if(Verbose)
    fprintf(stderr,": start time %s, %'d samp/s, RF LO %'.1lf Hz\n",lltime(status.timestamp),status.samprate,status.frequency);


  struct rtp_header rtp_header;
  memset(&rtp_header,0,sizeof(rtp_header));
  rtp_header.version = RTP_VERS;
  rtp_header.type = PCM_STEREO_PT;
  
  struct timeval start_time;
  gettimeofday(&start_time,NULL);

  rtp_header.ssrc = Rtp_state.ssrc;
  
  // microsec between packets. Double precision is used to avoid small errors that could
  // accumulate over time
  double dt = (1000000. * blocksize) / status.samprate;
  // Microseconds since start for next scheduled transmission; will transmit first immediately
  double sked_time = 0;
  
  while(1){
    rtp_header.seq = Rtp_state.seq++;
    rtp_header.timestamp = Rtp_state.timestamp;
    Rtp_state.timestamp += blocksize;
    
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
    unsigned char output_buffer[4*blocksize + 256]; // will this allow for largest possible RTP header??
    unsigned char *dp = output_buffer;
    dp = hton_rtp(dp,&rtp_header);


    if(pipefill(fd,dp,4*blocksize) <= 0)
      break;

    signed short *sp = (signed short *)dp;
    float p = 0;
    for(int n=0; n < 2*blocksize; n ++){
      p += (float)(*sp) * (float)(*sp);
      *sp = htons(*sp);
      sp++;
    }
    Power = p / (32767. * 32767. * blocksize);

    dp = (unsigned char *)sp;

    int length = dp - output_buffer;
    if(send(sock,output_buffer,length,0) == -1)
      perror("send");
    
    Rtp_state.packets++;
    // Update time of next scheduled transmission
    sked_time += dt;
    // Update nanosecond timestamp
    status.timestamp += blocksize * (long long)1e9 / status.samprate;
  }
  return 0;
}



struct option const Options[] =
  {
   {"iface", required_argument, NULL, 'A'},
   {"pcm-out", required_argument, NULL, 'D'},
   {"status-out", required_argument, NULL, 'R'},
   {"ssrc", required_argument, NULL, 'S'},
   {"ttl", required_argument, NULL, 'T'},
   {"blocksize", required_argument, NULL, 'b'},
   {"frequency", required_argument, NULL, 'f'},
   {"verbose", no_argument, NULL, 'v'},
   {"samprate", required_argument, NULL, 'r'},
   {NULL, 0, NULL, 0},
  };
char const Optstring[] = "A:D:R:S:T:b:f:vr:";


int main(int argc,char *argv[]){
#if 0 // Better done manually?
  // if we have root, up our priority and drop privileges
  int prio = getpriority(PRIO_PROCESS,0);
  prio = setpriority(PRIO_PROCESS,0,prio - 10);

  // Quickly drop root if we have it
  // The sooner we do this, the fewer options there are for abuse
  if(seteuid(getuid()) != 0)
    perror("seteuid");
#endif

  {
    char *locale = getenv("LANG");
    if(locale == NULL)
      locale = "en_US.UTF-8";
    
    setlocale(LC_ALL,locale);
  }
  {
    struct timeval start_time;
    gettimeofday(&start_time,NULL);
    Rtp_state.ssrc = start_time.tv_sec; // Default, can be overridden
  }
  int c;
  while((c = getopt_long(argc,argv,Optstring,Options,NULL)) != -1){
    switch(c){
    case 'A':
      Default_mcast_iface = optarg;
      break;
    case 'D':
      Rtp_sock = setup_mcast(optarg,(struct sockaddr *)&Output_data_dest_address,1,Mcast_ttl,0);
      if(Rtp_sock == -1){
	fprintf(stderr,"Can't create multicast socket: %s\n",strerror(errno));
	exit(1);
      }
      socklen_t len = sizeof(Output_data_source_address);
      getsockname(Rtp_sock,(struct sockaddr *)&Output_data_source_address,&len);
      break;
    case 'R':
      Status_sock = setup_mcast(optarg,(struct sockaddr *)&Output_metadata_dest_address,1,Mcast_ttl,2); // For output
      if(Status_sock <= 0){
	fprintf(stderr,"Can't create multicast socket: %s\n",strerror(errno));
	exit(1);
      }
      // Set up new control socket on port 5006
      Nctl_sock = setup_mcast(NULL,(struct sockaddr *)&Output_metadata_dest_address,0,Mcast_ttl,2); // For input
      if(Nctl_sock <= 0){
	fprintf(stderr,"Can't create multicast socket: %s\n",strerror(errno));
	exit(1);
      }
      break;
    case 'r':
      Samprate = strtol(optarg,NULL,0);
      break;
    case 'S':
      Rtp_state.ssrc = strtol(optarg,NULL,0);
      break;
    case 'T':
      Mcast_ttl = strtol(optarg,NULL,0);
      break;
    case 'v':
      Verbose++;
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

  if(Rtp_sock == -1){
    fprintf(stderr,"No PCM destination supplied (-D)\n");
    exit(1);
  }

  signal(SIGPIPE,SIG_IGN);

  pthread_t status;
  void *ncmd(void *);
  pthread_create(&status,NULL,ncmd,NULL);

  if(optind == argc){
    // No file arguments, read from stdin
    if(Verbose)
      fprintf(stderr,"Transmitting from stdin");
    playfile(Rtp_sock,0,Blocksize);
  } else {
    for(int i=optind;i<argc;i++){
      int fd;
      if((fd = open(argv[i],O_RDONLY)) == -1){
	fprintf(stderr,"Can't read %s; ",argv[i]);
	perror("");
	continue;
      }
      if(Verbose)
	fprintf(stderr,"Transmitting %s",argv[i]);
      playfile(Rtp_sock,fd,Blocksize);
      close(fd);
      fd = -1;
    }
  }
  close(Rtp_sock);
  Rtp_sock = -1;
  exit(0);
}

// Thread to send metadata and process commands
void *ncmd(void *arg){
  pthread_setname("iqsendcmd");
  assert(arg != NULL);
  
  memset(State,0,sizeof(State));

  // Set up status socket on port 5006
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000; // 100 ms

  if(setsockopt(Nctl_sock,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv))){
    perror("ncmd setsockopt");
    return NULL;
  }
  int counter = 0;
  while(1){
    unsigned char buffer[Bufsize];
    memset(buffer,0,sizeof(buffer));
    int length = recv(Nctl_sock,buffer,sizeof(buffer),0); // Waits up to 100 ms for command
    if(length > 0){

      // Parse entries
      unsigned char *cp = buffer;

      int cr = *cp++; // Command/response
      if(cr == 0)
	continue; // Ignore our own status messages
      Commands++;
      //      decode_iqplay_commands(sdr,cp,length-1);      // Implement later
      counter = 0; // Respond with full status
    }
    Output_metadata_packets++;
    send_iqplay_status(counter == 0);
    if(counter-- <= 0)
      counter = 10;
  }
}


void send_iqplay_status(int full){
  unsigned char packet[2048],*bp;
  memset(packet,0,sizeof(packet));
  bp = packet;
  
  *bp++ = 0; // command/response = response
  //  encode_int32(&bp,COMMAND_TAG,...);
  encode_int64(&bp,COMMANDS,Commands);
  
  struct timeval tp;
  gettimeofday(&tp,NULL);
  // Timestamp is in nanoseconds for futureproofing, but time of day is only available in microsec
  long long timestamp = ((tp.tv_sec - UNIX_EPOCH + GPS_UTC_OFFSET) * 1000000LL + tp.tv_usec) * 1000LL;
  encode_int64(&bp,GPS_TIME,timestamp);

  if(Description)
    encode_string(&bp,DESCRIPTION,Description,strlen(Description));
  
  encode_socket(&bp,OUTPUT_DATA_SOURCE_SOCKET,&Output_data_source_address);
  encode_socket(&bp,OUTPUT_DATA_DEST_SOCKET,&Output_data_dest_address);
  encode_int32(&bp,OUTPUT_SSRC,Rtp_state.ssrc);
  encode_byte(&bp,OUTPUT_TTL,Mcast_ttl);
  encode_int32(&bp,OUTPUT_SAMPRATE,Samprate);
  encode_int64(&bp,OUTPUT_DATA_PACKETS,Rtp_state.packets);
  encode_int64(&bp,OUTPUT_METADATA_PACKETS,Output_metadata_packets);
  
  // Front end
  encode_byte(&bp,DIRECT_CONVERSION,0);
  encode_float(&bp,GAIN,0.0); 
  
  // Tuning
  encode_double(&bp,RADIO_FREQUENCY,Default_frequency);
  
  // Filtering
  encode_float(&bp,OUTPUT_LEVEL,power2dB(Power));

  encode_byte(&bp,DEMOD_TYPE,0); // Actually LINEAR_MODE
  encode_int32(&bp,OUTPUT_CHANNELS,2);

  encode_eol(&bp);
  assert(bp - packet < sizeof(packet));
  
  int len = compact_packet(&State[0],packet,full);
  send(Status_sock,packet,len,0);
}
