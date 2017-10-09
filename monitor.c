// $Id: monitor.c,v 1.21 2017/09/28 22:02:37 karn Exp karn $
// Listen to multicast, send PCM audio to Linux ALSA driver
#define _GNU_SOURCE 1
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <limits.h>
#include <string.h>
#include <opus/opus.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/resource.h>
#include <netdb.h>
#include <portaudio.h>
#include <ncurses.h>
#include <locale.h>

#include "rtp.h"
#include "dsp.h"
#include "multicast.h"

// Global config variables
char *Mcast_address_text = "239.2.1.1";     // Multicast address we're listening to
char Audiodev[256];           // Name of audio device; empty means portaudio's default
const int Slop = 480;         // Playout delay in samples to add after an underrun
#define Aud_bufsize (1<<18)     // make this a power of 2 for efficiency
const int Bufsize = 8192;     // Maximum samples/words per RTP packet - must be bigger than Ethernet MTU
const int Samprate = 48000;   // Too hard to handle other sample rates right now
int List_audio;               // List audio output devices and exit
int Verbose;                  // Verbosity flag (currently unused)
#define Hashchains 16         // Make this a power of 2 for efficiency
int Update_interval = 1000000; // Time in usec between display updates
int Clear_interval = 20;      // Time in sec until inactive entry is cleared

pthread_t Display_thread;     // Display thread descriptor
int Input_fd = -1;            // Multicast receive socket
struct audio *Audio[Hashchains];     // Hash chains for session structures
PaStream *Pa_Stream;          // Portaudio stream handle
int inDevNum;                 // Portaudio's audio output device index

// The portaudio callback continuously plays out this single buffer, into which multiple streams sum their audio
// and the callback zeroes out each sample as played
float Audio_buffer[Aud_bufsize]; // Audio playout buffer
volatile int Read_ptr;           // read_pointer, modified in callback (hence volatile)
pthread_mutex_t Buffer_mutex;


struct audio {
  struct audio *prev;       // Linked list pointers
  struct audio *next; 
  uint32_t ssrc;            // RTP Sending Source ID
  int eseq;                 // Next expected RTP sequence number
  int etime;                // Next expected RTP timestamp
  int type;                 // RTP type (10,11,20)
  
  struct sockaddr sender;
  char addr[NI_MAXHOST];    // RTP Sender IP address
  char port[NI_MAXSERV];    // RTP Sender source port
  OpusDecoder *opus;        // Opus codec decoder handle, if needed
  int opus_frame_size;      // Opus frame size in samples
  int channels;             // Channels (1 or 2)
  int opus_bandwidth;       // Opus stream audio bandwidth

  int write_ptr;   // Playout buffer write pointer
  PaTime last_write_time;

  unsigned long packets;    // RTP packets for this session
  unsigned long drops;      // Apparent rtp packet drops
  int age;                  // Display cycles since last active
  unsigned long underruns;  // Callback count of underruns (stereo samples) replaced with silence
  int hw_delay;             // Estimated playout delay in samples
};


// The Audio structures are accessed by both display() and main(), so protect them
pthread_mutex_t Audio_mutex;

void closedown(int);
void *display(void *);
struct audio *lookup_session(const struct sockaddr *,uint32_t);
struct audio *make_session(struct sockaddr const *r,uint32_t,uint16_t,uint32_t);
int close_session(struct audio *);
static int pa_callback(const void *,void *,unsigned long,const PaStreamCallbackTimeInfo*,PaStreamCallbackFlags,void *);
int write_is_ahead(int write_ptr,int read_ptr);


int main(int argc,char * const argv[]){
  // Try to improve our priority
  int prio = getpriority(PRIO_PROCESS,0);
  prio = setpriority(PRIO_PROCESS,0,prio - 5);

  // Drop root if we have it
  seteuid(getuid());

  setlocale(LC_ALL,getenv("LANG"));

  int c;
  while((c = getopt(argc,argv,"a:S:I:vL")) != EOF){
    switch(c){
    case 'L':
      List_audio++;
      break;
    case 'a':
      strncpy(Audiodev,optarg,sizeof(Audiodev));
      break;
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

  PaError r = Pa_Initialize();
  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));
    return r;
  }

  if(List_audio){
    // On stdout, not stderr, so we can toss ALSA's noisy error messages
    printf("Audio output devices:\n");
    int numDevices = Pa_GetDeviceCount();
    for(int inDevNum=0; inDevNum < numDevices; inDevNum++){
      const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(inDevNum);
      printf("%s\n",deviceInfo->name);
    }
    exit(0);
  }

  if(strlen(Audiodev) == 0){
    // not specified; use default
    inDevNum = Pa_GetDefaultOutputDevice();
  } else {
    // Find requested audio device in the list
    int numDevices = Pa_GetDeviceCount();
    
    for(inDevNum=0; inDevNum < numDevices; inDevNum++){
      const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(inDevNum);
      if(strcmp(deviceInfo->name,Audiodev) == 0)
	break;
    }
  }
  if(inDevNum == paNoDevice){
    fprintf(stderr,"Portaudio: no available devices\n");
    return -1;
  }
  // Set up multicast input
  Input_fd = setup_mcast(Mcast_address_text,0);
  if(Input_fd == -1){
    fprintf(stderr,"Can't set up input\n");
    exit(1);
  }
  struct iovec iovec[2];
  struct rtp_header rtp;
  int16_t data[Bufsize];
  
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

  // Temporarily put socket in nonblocking state
  // and flush any pending packets to avoid unnecessary long buffer delays
  int flags;
  if((flags = fcntl(Input_fd,F_GETFL)) != -1){
    flags |= O_NONBLOCK;
    if(fcntl(Input_fd,F_SETFL,flags) == 0){
      int flushed = 0;
      while(recvmsg(Input_fd,&message,0) >= 0)
	flushed++;
      flags &= ~O_NONBLOCK;
      fcntl(Input_fd,F_SETFL,flags);
    }
  }

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);

  pthread_mutex_init(&Buffer_mutex,NULL);
  pthread_mutex_init(&Audio_mutex,NULL);
  pthread_create(&Display_thread,NULL,display,NULL);

  // Create and start portaudio stream
  PaStreamParameters outputParameters;
  memset(&outputParameters,0,sizeof(outputParameters));
  outputParameters.channelCount = 2;
  outputParameters.device = inDevNum;
  outputParameters.sampleFormat = paFloat32;
  
#if 0
  r = Pa_OpenStream(&Pa_Stream,NULL,&outputParameters,Samprate,paFramesPerBufferUnspecified,
		    paPrimeOutputBuffersUsingStreamCallback,pa_callback,NULL);
#else
  r = Pa_OpenStream(&Pa_Stream,NULL,&outputParameters,Samprate,paFramesPerBufferUnspecified,
		    0,pa_callback,NULL);
#endif      
  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));      
    exit(1);
  }
  r = Pa_StartStream(Pa_Stream);
  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));
    exit(1);
  }


  while(1){
    ssize_t size;

    if((size = recvmsg(Input_fd,&message,0)) < sizeof(rtp)){
      usleep(500); // Avoid tight loop
      continue; // Too small to be valid RTP
    }
    // To host order
    rtp.ssrc = ntohl(rtp.ssrc);
    rtp.seq = ntohs(rtp.seq);
    rtp.timestamp = ntohl(rtp.timestamp);

    pthread_mutex_lock(&Audio_mutex); // Keep display thread from modifying the table

    struct audio *sp = lookup_session(&sender,rtp.ssrc);
    if(sp == NULL){
      // Not found
      if((sp = make_session(&sender,rtp.ssrc,rtp.seq,rtp.timestamp)) == NULL){
	fprintf(stderr,"No room!!\n");
	goto endloop;
      }
      getnameinfo((struct sockaddr *)&sender,sizeof(sender),sp->addr,sizeof(sp->addr),
		    sp->port,sizeof(sp->port),NI_NOFQDN|NI_DGRAM|NI_NUMERICHOST);
      sp->write_ptr = Read_ptr;

    }
    sp->age = 0;
    int drop = 0;
    if(rtp.seq != sp->eseq){
      int const diff = (int)(rtp.seq - sp->eseq);
      //      fprintf(stderr,"ssrc %lx: expected %d got %d\n",(unsigned long)rtp.ssrc,sp->eseq,rtp.seq);
      if(diff < 0 && diff > -10)
	goto endloop;	// Drop probable duplicate
      drop = diff; // Apparent # packets dropped
      sp->drops += abs(drop);
    }
    sp->eseq = (rtp.seq + 1) & 0xffff;
    size -= sizeof(rtp); // Bytes in payload
    sp->packets++;

    int samples = 0;
    sp->type = rtp.mpt;

    switch(rtp.mpt){
    case 10: // Stereo
      sp->channels = 2;
      samples = size / 4;  // # 32-bit word samples
      pthread_mutex_lock(&Buffer_mutex);
      if(Pa_GetStreamTime(Pa_Stream) > sp->last_write_time + 0.5 || !write_is_ahead(sp->write_ptr,Read_ptr)){
	// Past deadline, skip forward
	sp->write_ptr = (Read_ptr + Slop) % Aud_bufsize;
	sp->underruns++;
      }
      for(int i=0;i<2*samples;i++){
	Audio_buffer[sp->write_ptr++] += ntohs(data[i]); // RTP profile specifies big-endian samples
	if(sp->write_ptr >= Aud_bufsize)
	  sp->write_ptr -= Aud_bufsize;
      }
      sp->hw_delay = (sp->write_ptr - Read_ptr + Aud_bufsize) % Aud_bufsize;
      sp->last_write_time = Pa_GetStreamTime(Pa_Stream);
      pthread_mutex_unlock(&Buffer_mutex);
      break;
    case 11: // Mono; send to both stereo channels
      sp->channels = 1;
      samples = size / 2;
      pthread_mutex_lock(&Buffer_mutex);
      if(Pa_GetStreamTime(Pa_Stream) > sp->last_write_time + 0.5 || !write_is_ahead(sp->write_ptr,Read_ptr)){
	// Past deadline, skip forward
	sp->write_ptr = (Read_ptr + Slop) % Aud_bufsize;
	sp->underruns++;
      }
      for(int i=0;i<samples;i++){
	Audio_buffer[sp->write_ptr++] += ntohs(data[i]);
	Audio_buffer[sp->write_ptr++] += ntohs(data[i]);
	if(sp->write_ptr >= Aud_bufsize)
	  sp->write_ptr -= Aud_bufsize;
      }
      sp->hw_delay = (sp->write_ptr - Read_ptr + Aud_bufsize) % Aud_bufsize;
      sp->last_write_time = Pa_GetStreamTime(Pa_Stream);
      pthread_mutex_unlock(&Buffer_mutex);
      break;
    case 20: // Opus codec decode - arbitrary choice
      if(sp->opus == NULL){ // Create if it doesn't already exist
	int error;
	sp->opus = opus_decoder_create(Samprate,2,&error);
	if(sp->opus == NULL)
	  fprintf(stderr,"Opus decoder error %d\n",error);
	break;
      }
      sp->channels = opus_packet_get_nb_channels((unsigned char *)data);
      sp->opus_bandwidth = opus_packet_get_bandwidth((unsigned char *)data);
      int nb_frames = opus_packet_get_nb_frames((unsigned char *)data,size);
      sp->opus_frame_size = nb_frames * opus_packet_get_samples_per_frame((unsigned char *)data,Samprate);

      {
	float outsamps[2*Bufsize];
	if(drop != 0){
	  // previous packet(s) dropped; have codec tell us how much silence to emit
	  drop = opus_decode_float(sp->opus,NULL,rtp.timestamp - sp->etime,outsamps,Bufsize,0);
	  sp->write_ptr += (2 * drop);
	  sp->write_ptr %= Aud_bufsize;
	}
	samples = opus_decode_float(sp->opus,(unsigned char *)data,size,outsamps,Bufsize,0);
	pthread_mutex_lock(&Buffer_mutex);
	if(Pa_GetStreamTime(Pa_Stream) > sp->last_write_time + 0.5 || !write_is_ahead(sp->write_ptr,Read_ptr)){
	  // Past deadline, skip forward
	  sp->underruns++;
	  sp->write_ptr = (Read_ptr + Slop) % Aud_bufsize;
	}
	for(int i=0; i < samples; i++){
	  Audio_buffer[sp->write_ptr++] += outsamps[2*i];
	  Audio_buffer[sp->write_ptr++] += outsamps[2*i+1];
	  if(sp->write_ptr >= Aud_bufsize)
	    sp->write_ptr -= Aud_bufsize;
	}
	sp->hw_delay = (sp->write_ptr - Read_ptr + Aud_bufsize) % Aud_bufsize;
	sp->last_write_time = Pa_GetStreamTime(Pa_Stream);
	pthread_mutex_unlock(&Buffer_mutex);
      }
      break;
    default:
      break; // ignore
    }

    sp->etime = rtp.timestamp + samples;

  endloop:;
    pthread_mutex_unlock(&Audio_mutex); // Give display thread a chance
  }
  pthread_cancel(Display_thread);
  endwin();
  exit(0);
}

// Use ncurses to display streams; age out unused ones
void *display(void *arg){
  initscr();
  // WINDOW * const mainscr = newwin(25,110,0,0);
  WINDOW * const mainscr = stdscr;
  int const agelimit = Clear_interval * 1000000 / Update_interval;

  while(1){
    int row = 1;
    wmove(mainscr,row,0);
    wclrtobot(mainscr);
    mvwprintw(mainscr,row++,1,"Type        channels   BW      SSRC     Packets     Drops   Underruns   Delay   Source");
    pthread_mutex_lock(&Audio_mutex);
    for(int i=0;i<Hashchains;i++){
      struct audio *nextsp; // Save in case we close the current one
      for(struct audio *sp = Audio[i]; sp != NULL; sp = nextsp){
	nextsp = sp->next;
	if(++sp->age > agelimit){
	  // Age out old session
	  close_session(sp);
	  continue;
	}
	int bw; // Audio bandwidth (not bitrate) in kHz
	char *type,typebuf[30];
	switch(sp->type){
	case 10:
	case 11:
	  type = "PCM";
	  bw = Samprate / 2000;
	  break;
	case 20:
	  switch(sp->opus_bandwidth){
	  case OPUS_BANDWIDTH_NARROWBAND:
	    bw = 4;
	    break;
	  case OPUS_BANDWIDTH_MEDIUMBAND:
	    bw = 6;
	    break;
	  case OPUS_BANDWIDTH_WIDEBAND:
	    bw = 8;
	    break;
	  case OPUS_BANDWIDTH_SUPERWIDEBAND:
	    bw = 12;
	    break;
	  case OPUS_BANDWIDTH_FULLBAND:
	    bw = 20;
	    break;
	  case OPUS_INVALID_PACKET:
	    bw = 0;
	    break;
	  }
	  snprintf(typebuf,sizeof(typebuf),"Opus %.1lf ms",1000.*sp->opus_frame_size/Samprate);
	  type = typebuf;
	  break;
	default:
	  snprintf(typebuf,sizeof(typebuf),"%d",sp->type);
	  bw = 0; // Unknown
	  type = typebuf;
	  break;
	}
	if(sp->age < 5)
	  wattr_on(mainscr,A_BOLD,NULL); // Embolden active streams
       	mvwprintw(mainscr,row++,1,"%-15s%5d%5d%10lx%'12lu%'10u%'12lu%'8.3lf   %s:%s",
		  type,sp->channels,bw,sp->ssrc,sp->packets,sp->drops,sp->underruns,(double)0.5*sp->hw_delay/Samprate,sp->addr,sp->port);
	wattr_off(mainscr,A_BOLD,NULL);
      }
    }
    pthread_mutex_unlock(&Audio_mutex);
    // Draw the box and banner last, to avoid the wclrtobot() calls
    //    box(mainscr,0,0);
    mvwprintw(mainscr,0,15,"KA9Q Multicast Audio Monitor - %s",Mcast_address_text);
    wnoutrefresh(mainscr);
    doupdate();
    usleep(Update_interval);
  }
}

struct audio *lookup_session(const struct sockaddr *sender,const uint32_t ssrc){
  // Walk hash chain
  struct audio *sp;
  for(sp = Audio[ssrc % Hashchains]; sp != NULL; sp = sp->next){
    if(sp->ssrc == ssrc && memcmp(&sp->sender,sender,sizeof(*sender)) == 0){
      // Found it
#if 0
      if(sp->prev != NULL){
	// Not at top of bucket chain; move it there
	if(sp->next != NULL)
	  sp->next->prev = sp->prev;

	sp->prev->next = sp->next;
	sp->prev = NULL;
	sp->next = Audio[ssrc % Hashchains];
	Audio[ssrc % Hashchains] = sp;
      }
#endif
      return sp;
    }
  }
  return NULL;
}
// Create a new session, partly initialize; ssrc is used as hash key
struct audio *make_session(struct sockaddr const *sender,uint32_t ssrc,uint16_t seq,uint32_t timestamp){
  struct audio *sp;

  if((sp = calloc(1,sizeof(*sp))) == NULL)
    return NULL; // Shouldn't happen on modern machines!
  
  // Initialize entry
  memcpy(&sp->sender,sender,sizeof(struct sockaddr));
  sp->ssrc = ssrc;
  sp->eseq = seq;
  sp->etime = timestamp;

  // Put at head of bucket chain
  sp->next = Audio[ssrc % Hashchains];
  if(sp->next != NULL)
    sp->next->prev = sp;
  Audio[ssrc % Hashchains] = sp;
  return sp;
}

int close_session(struct audio *sp){
  if(sp == NULL)
    return -1;
  
  if(sp->opus != NULL){
    opus_decoder_destroy(sp->opus);
    sp->opus = NULL;
  }
  // Remove from linked list
  if(sp->next != NULL)
    sp->next->prev = sp->prev;
  if(sp->prev != NULL)
    sp->prev->next = sp->next;
  else
    Audio[sp->ssrc % Hashchains] = sp->next;
  free(sp);
  return 0;
}
void closedown(int s){
  for(int i=0; i < Hashchains; i++){
    while(Audio[i] != NULL)
      close_session(Audio[i]);
  }
  Pa_Terminate();
  endwin();

  exit(0);
}

// Portaudio callback - transfer data (if any) to provided buffer
// When buffer is empty, pad with silence
static int pa_callback(const void *inputBuffer, void *outputBuffer,
		       unsigned long framesPerBuffer,
		       const PaStreamCallbackTimeInfo* timeInfo,
		       PaStreamCallbackFlags statusFlags,
		       void *userData){
  float *op = outputBuffer;
  if(op == NULL)
    return paAbort; // can this happen??
  

#if 1
  pthread_mutex_lock(&Buffer_mutex); // Protect Read_ptr
#endif
#if 1
  int samples_left = 2 * framesPerBuffer; // A stereo frame is two samples
  while(samples_left > 0){
    // chunk is the smaller of the samples needed and the amount available before wrap
    int chunk = Aud_bufsize - Read_ptr;
    if(chunk > samples_left)
      chunk = samples_left;
    memcpy(op,&Audio_buffer[Read_ptr],chunk * sizeof(*op));
    memset(&Audio_buffer[Read_ptr],0,chunk * sizeof(*op)); // Zero buffer just read
    op += chunk;
    samples_left -= chunk;
    // Update the read pointer, try to do it atomically in case we can't lock
    Read_ptr = (Read_ptr + chunk) & (Aud_bufsize - 1); // Assumes Aud_bufsize is power of 2!!
  }
#else
  // Old slow code
  for(int i=0; i<framesPerBuffer; i++){
    *op++ = Audio_buffer[Read_ptr];
    Audio_buffer[Read_ptr++] = 0;
    *op++ = Audio_buffer[Read_ptr];
    Audio_buffer[Read_ptr++] = 0;      // zero out data just read
    if(Read_ptr >= Aud_bufsize)
      Read_ptr -= Aud_bufsize;
  }
#endif
#if 1
  pthread_mutex_unlock(&Buffer_mutex);
#endif
  return paContinue;
}

// Check for underrunin
int write_is_ahead(int write_ptr,int read_ptr){
  int n = (write_ptr - read_ptr + Aud_bufsize) % Aud_bufsize;
  if(n < Aud_bufsize/2)
    return 1; // OK
  else
    return 0; // Underrun has occurred
}
