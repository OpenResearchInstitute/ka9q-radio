// $Id: monitor.c,v 1.37 2018/03/23 22:30:12 karn Exp karn $
// Listen to multicast, send PCM audio to Linux ALSA driver
// Copyright 2018 Phil Karn, KA9Q
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
#include <sys/select.h>
#include <netdb.h>
#include <portaudio.h>
#include <ncurses.h>
#include <locale.h>
#include <errno.h>

#include "misc.h"
#include "multicast.h"

// Global config variables
char *Mcast_address_text[10]; // Multicast address(es) we're listening to
char Audiodev[256];           // Name of audio device; empty means portaudio's default
const int Bufsize = 16384;    // Maximum bytes per RTP packet - must be bigger than Ethernet MTU
const int Samprate = 48000;   // Too hard to handle other sample rates right now
int List_audio;               // List audio output devices and exit
int Verbose;                  // Verbosity flag (currently unused)
int Quiet;                    // Disable curses
int Update_interval = 100;    // Time in ms between display updates
int Age_limit = 1000;          // Delete after 100 sec
int Highlight_limit = 10;     // Dim after 1 second
#define NCHAN 2               // 2 channels (stereo)
#define SAMPPCALLBACK 480     // 10 ms @ 48 kHz
#define BUFFERSIZE (524288) // about 5.5 sec at 48 kHz stereo


int Nfds;                    // Number of streams
struct audio *Audio;          // Link to head of session structure chain
PaStream *Pa_Stream;          // Portaudio stream handle
int inDevNum;                 // Portaudio's audio output device index

// The portaudio callback continuously plays out this single buffer, into which multiple streams sum their audio
// and the callback zeroes out each sample as played
float const SCALE = 1./SHRT_MAX;
WINDOW *Mainscr;

pthread_cond_t Upcall_cond;
pthread_mutex_t Upcall_mutex;
pthread_mutex_t Buffer_mutex;


// Incoming RTP packets
struct packet {
  struct packet *next;
  struct rtp_header rtp;
  unsigned char data[Bufsize];
  int len;
};



struct audio {
  struct audio *prev;       // Linked list pointers
  struct audio *next; 
  uint32_t ssrc;            // RTP Sending Source ID
  uint16_t eseq;                 // Next expected RTP sequence number
  int etimestamp;           // Next expected RTP timestamp
  int type;                 // RTP type (10,11,20)
  
  struct packet *queue;

  float gain;               // Gain for this channel; 1 -> 0 db (nominal)

  struct sockaddr sender;
  char *dest;
  char addr[NI_MAXHOST];    // RTP Sender IP address
  char port[NI_MAXSERV];    // RTP Sender source port
  OpusDecoder *opus;        // Opus codec decoder handle, if needed
  int opus_frame_size;      // Opus frame size in samples
  int channels;             // Channels (1 or 2)
  int opus_bandwidth;       // Opus stream audio bandwidth

  pthread_t task;
  pthread_mutex_t qmutex;
  pthread_cond_t qcond;

  unsigned long packets;    // RTP packets for this session
  unsigned long drops;      // Apparent rtp packet drops
  unsigned long invalids;   // Unknown RTP type
  unsigned long empties;    // RTP but no data
  unsigned long dupes;      // Duplicate or old serial numbers
  int age;                  // Display cycles since last active
  unsigned long lates;      // Callback count of underruns (stereo samples) replaced with silence
  int hw_delay;             // Estimated playout delay in samples

  float output_buffer[BUFFERSIZE];
  int wptr; // Write pointer into output_buffer
  int rptr; // Read pointer into output buffer
  int offset; // playout delay offset
  int pause;  // Playback paused
};
struct audio *Current = NULL;

void closedown(int);
void display();
struct audio *lookup_session(const struct sockaddr *,uint32_t);
struct audio *create_session(struct sockaddr const *,uint32_t);
int close_session(struct audio *);
static int pa_callback(const void *,void *,unsigned long,const PaStreamCallbackTimeInfo*,PaStreamCallbackFlags,void *);
void *opus_task(void *x);

// Add a and b, modulo buffersize
// Result is always zero or positive even if b is negative
static inline int addmod(int a,int b){
  return (a + b) & (BUFFERSIZE-1);
}

// Subtract b from a for ordering comparisons
// result is between -BUFFERSIZE/2 and +BUFFERSIZE/2
static int submod(int a,int b){
  a = addmod(a,-b);
  if(a > BUFFERSIZE/2)
    a -= BUFFERSIZE;
  if(a < -BUFFERSIZE/2)
    a += BUFFERSIZE;
  return a;
}




int main(int argc,char * const argv[]){
  // Try to improve our priority
  int prio = getpriority(PRIO_PROCESS,0);
  prio = setpriority(PRIO_PROCESS,0,prio - 15);

  // Drop root if we have it
  seteuid(getuid());

  setlocale(LC_ALL,getenv("LANG"));

  int c;
  while((c = getopt(argc,argv,"a:S:I:vLE:A:q")) != EOF){
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
      Mcast_address_text[Nfds++] = optarg;
      break;
    case 'E':
      Highlight_limit = strtol(optarg,NULL,0);
      break;
    case 'A':
      Age_limit = strtol(optarg,NULL,0);
      break;
    case 'q': // No ncurses
      Quiet++;
      break;
    default:
      fprintf(stderr,"Usage: %s [-v] [-I mcast_address]\n",argv[0]);
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

  if(Nfds == 0){
    fprintf(stderr,"At least one -I option required\n");
    exit(1);
  }


  struct iovec iovec[2]; // These contents are rewritten before every recvmsg()
  
  struct msghdr message;
  struct sockaddr sender;
  message.msg_name = &sender;
  message.msg_namelen = sizeof(sender);
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 2;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  // Set up multicast input
  fd_set fdset_template; // Mask for select()
  FD_ZERO(&fdset_template);
  int max_fd = 2;        // Highest number fd for select()
  int input_fd[Nfds];    // Multicast receive sockets

  for(int i=0;i<Nfds;i++){
    input_fd[i] = setup_mcast(Mcast_address_text[i],0);
    if(input_fd[i] == -1){
      fprintf(stderr,"Can't set up input %s\n",Mcast_address_text[i]);
      continue;
    }
    if(input_fd[i] > max_fd)
      max_fd = input_fd[i];
    FD_SET(input_fd[i],&fdset_template);
  }
  // Create and start portaudio stream.
  // Runs continuously, playing silence until audio arrives.
  // This allows multiple streams to be played on hosts that only support one
  PaStreamParameters outputParameters;
  memset(&outputParameters,0,sizeof(outputParameters));
  outputParameters.channelCount = NCHAN;
  outputParameters.device = inDevNum;
  outputParameters.sampleFormat = paFloat32;
  outputParameters.suggestedLatency = 0.010; // 0 doesn't seem to be a good value on OSX, lots of underruns and stutters
  
  r = Pa_OpenStream(&Pa_Stream,
		    NULL,
		    &outputParameters,
		    Samprate,
                    SAMPPCALLBACK,		    //paFramesPerBufferUnspecified,
		    0,
		    pa_callback,
		    NULL);

  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));      
    exit(1);
  }

  pthread_cond_init(&Upcall_cond,NULL);
  pthread_mutex_init(&Upcall_mutex,NULL);
  pthread_mutex_init(&Buffer_mutex,NULL);

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);

  if(!Quiet){
    initscr();
    keypad(stdscr,TRUE);
    timeout(0); // Nonblocking
    cbreak();
    noecho();
    
    Mainscr = stdscr;
  }
  struct timeval last_update_time;
  gettimeofday(&last_update_time,NULL);
  // Do this last since the upcall will come quickly
  r = Pa_StartStream(Pa_Stream);
  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));
    exit(1);
  }


  while(1){
    struct timeval timeout;    
    if(!Quiet){
      struct timeval tp;
      gettimeofday(&tp,NULL);
      // Time since last update in milliseconds
      int interval = (tp.tv_sec - last_update_time.tv_sec)*1000 + (tp.tv_usec - last_update_time.tv_usec)/1000.;
      if(interval >= Update_interval){
	last_update_time = tp;
	display();
      }
      timeout.tv_sec = 0;
      timeout.tv_usec = 1000 * Update_interval;
    } else {
      timeout.tv_sec = 0;
      timeout.tv_usec = 0;
    }
    fd_set fdset;
    fdset = fdset_template;
    //    FD_COPY(&fdset_template,&fdset); // not on Linux; simple assign seems to work
    int s = select(max_fd+1,& fdset,NULL,NULL,&timeout);
    if(s < 0 && errno != EAGAIN && errno != EINTR)
      break;
    if(s == 0)
      continue;

    for(int fd_index = 0;fd_index < Nfds;fd_index++){
      if(!FD_ISSET(input_fd[fd_index],&fdset))
	continue;

      struct packet *pkt = malloc(sizeof(*pkt));
      pkt->next = NULL;
      // message points to iovec[]
      iovec[0].iov_base = &pkt->rtp;
      iovec[0].iov_len = sizeof(pkt->rtp);
      iovec[1].iov_base = &pkt->data;
      iovec[1].iov_len = sizeof(pkt->data);

      int size = recvmsg(input_fd[fd_index],&message,0);
      if(size == -1){
	if(errno != EINTR){ // Happens routinely
	  perror("recvmsg");
 	  usleep(1000);
	}
	free(pkt);
	continue;
      }
      if(size < sizeof(pkt->rtp)){
	usleep(500); // Avoid tight loop
	free(pkt);
	continue; // Too small to be valid RTP
      }
      // To host order
      pkt->rtp.ssrc = ntohl(pkt->rtp.ssrc);
      pkt->rtp.seq = ntohs(pkt->rtp.seq);
      pkt->rtp.timestamp = ntohl(pkt->rtp.timestamp);
      pkt->len = size - sizeof(pkt->rtp); // Bytes in payload
      
      if(pkt->rtp.mpt != 10 && pkt->rtp.mpt != 20 && pkt->rtp.mpt != 11){ // 1 byte, no need to byte swap
	free(pkt);
	continue; // Discard unknown RTP types to avoid polluting session table
      }      
      struct audio *sp = lookup_session(&sender,pkt->rtp.ssrc);
      if(sp == NULL){
	// Not found
	if((sp = create_session(&sender,pkt->rtp.ssrc)) == NULL){
	  fprintf(stderr,"No room!!\n");
	  free(pkt);
	  continue;
	}
	getnameinfo((struct sockaddr *)&sender,sizeof(sender),sp->addr,sizeof(sp->addr),
		    //		    sp->port,sizeof(sp->port),NI_NOFQDN|NI_DGRAM|NI_NUMERICHOST);
		    sp->port,sizeof(sp->port),NI_NOFQDN|NI_DGRAM);
	sp->dest = Mcast_address_text[fd_index];
	sp->etimestamp = pkt->rtp.timestamp;
	sp->eseq = pkt->rtp.seq;
	
	sp->gain = 1; // 0 dB by default
	sp->dupes = 0;

	pthread_mutex_init(&sp->qmutex,NULL);
	pthread_cond_init(&sp->qcond,NULL);
	if(pthread_create(&sp->task,NULL,opus_task,sp) == -1){
	  perror("pthread_create");
	  close_session(sp);
	  free(pkt);
	  continue;
	}
      }
      sp->age = 0;
      sp->packets++;
      sp->type = pkt->rtp.mpt;
      if((signed short)(pkt->rtp.seq - sp->eseq) < 0){
	sp->dupes++;
	free(pkt);
	continue;      // Discard old duplicate
      }
      
      // Sort onto list
      struct packet *q_prev = NULL;
      struct packet *qe = NULL;
      pthread_mutex_lock(&sp->qmutex);
      
      for(qe = sp->queue;qe != NULL; q_prev = qe,qe = qe->next){
	if(pkt->rtp.seq < qe->rtp.seq)
	  break;
      }
      pkt->next = qe;
      if(q_prev != NULL)
	q_prev->next = pkt;
      else
	sp->queue = pkt; // Front of list
      pkt = NULL; // ensure it can't be reused
      // wake up decoder thread
      pthread_cond_broadcast(&sp->qcond);
      pthread_mutex_unlock(&sp->qmutex);
    }      
  }
  echo();
  nocbreak();
  endwin();
  exit(0);
}

// Use ncurses to display streams; age out unused ones
void display(){

    int row = 1;
    wmove(Mainscr,row,0);
    wclrtobot(Mainscr);
    mvwprintw(Mainscr,row++,0,"Type        chans BW   Gain      SSRC     Packets  Dupes  Drops     Lates  Queue Source                Dest");
    // Age out idle sessions
    struct audio *nextsp; // Save in case we close the current one
    for(struct audio *sp = Audio; sp != NULL; sp = nextsp){
      nextsp = sp->next;
      if(++sp->age > Age_limit){
	// Age out old session
	close_session(sp);
	if(Current == sp)
	  Current = Audio;
      }
    }

    for(struct audio *sp = Audio; sp != NULL; sp = sp->next){
      int bw = 0; // Audio bandwidth (not bitrate) in kHz
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
      wmove(Mainscr,row,1);
      wclrtoeol(Mainscr);
      if(sp->age < Highlight_limit)
	wattr_on(Mainscr,A_BOLD,NULL); // Embolden active streams

      char source_text[256];
      int qlen = submod(sp->wptr,addmod(sp->rptr,sp->offset));
      double qdelay = (double)qlen / (NCHAN*Samprate);
      
      snprintf(source_text,sizeof(source_text),"%s:%s",sp->addr,sp->port);
      mvwprintw(Mainscr,row,0,"%-15s%2d%3d%+7.0lf%10x%'12lu%7lu%7lu%'10lu %6.3lf %-22s%-16s",
		type,
		sp->channels,
		bw,
		20*log10(sp->gain),
		sp->ssrc,
		sp->packets,
		sp->dupes,
		sp->drops,
		sp->lates,
		qdelay,
		source_text,
		sp->dest);
      wattr_off(Mainscr,A_BOLD,NULL);
      if(Current == NULL)
	Current = sp;
      if(sp == Current)
	mvwchgat(Mainscr,row,22,5,A_STANDOUT,0,NULL);

      row++;
    }
    move(row,1);
    clrtobot();
    mvwprintw(Mainscr,0,0,"KA9Q Multicast Audio Monitor:");
    for(int i=0;i<Nfds;i++)
      wprintw(Mainscr," %s",Mcast_address_text[i]);
    wprintw(Mainscr,"\n");
    wnoutrefresh(Mainscr);
    doupdate();
    if(Current != NULL){ // process commands only if there's something to act on
      int c = getch();
      switch(c){
      case EOF:
	break;
      case KEY_NPAGE:
      case '\t':
	if(Current->next != NULL)
	  Current = Current->next;
	else if(Audio != NULL)
	  Current = Audio; // Wrap around to top
	break;
      case KEY_BTAB:
      case KEY_PPAGE:
	if(Current->prev != NULL)
	  Current = Current->prev;
	break;
      case KEY_UP:
	Current->gain *= 1.122018454; // 1 dB
	break;
      case KEY_DOWN:
	Current->gain /= 1.122018454; // 1 dB
	break;
      case KEY_RIGHT:
	Current->offset = submod(Current->offset,NCHAN*48); // 1 ms
	break;
      case KEY_LEFT:
	Current->offset = addmod(Current->offset,NCHAN*48); // 1 ms
	break;
      case 'd':
	{
	  struct audio *next = Current->next;
	  close_session(Current);
	  Current = next;
	}
	break;
      default:
	break;
      }
    }
}



struct audio *lookup_session(const struct sockaddr *sender,const uint32_t ssrc){
  struct audio *sp;
  for(sp = Audio; sp != NULL; sp = sp->next){
    if(sp->ssrc == ssrc && memcmp(&sp->sender,sender,sizeof(*sender)) == 0){
      // Found it
#if 0  // Disable this - confuses user doing menu selections
      if(sp->prev != NULL){
	// Not at top of bucket chain; move it there
	if(sp->next != NULL)
	  sp->next->prev = sp->prev;

	sp->prev->next = sp->next;
	sp->prev = NULL;
	sp->next = Audio;
	Audio = sp;
      }
#endif
      return sp;
    }
  }
  return NULL;
}
// Create a new session, partly initialize
struct audio *create_session(struct sockaddr const *sender,uint32_t ssrc){
  struct audio *sp;

  if((sp = calloc(1,sizeof(*sp))) == NULL)
    return NULL; // Shouldn't happen on modern machines!
  
  // Initialize entry
  memcpy(&sp->sender,sender,sizeof(struct sockaddr));
  sp->ssrc = ssrc;
  // Put at head of bucket chain
  sp->next = Audio;
  if(sp->next != NULL)
    sp->next->prev = sp;
  Audio = sp;
  return sp;
}

int close_session(struct audio *sp){
  if(sp == NULL)
    return -1;
  
  pthread_cancel(sp->task);
  pthread_join(sp->task,NULL);

  // This isn't really right -- if the mutex is locked by the thread we're cancelling
  // this won't work if the mutex is the error-checking type. We should get the thread
  // itself to free the mutex
  if(pthread_mutex_trylock(&Upcall_mutex) == EBUSY)
    pthread_mutex_unlock(&Upcall_mutex); // Task is probably blocked here, so free lock
  pthread_mutex_destroy(&sp->qmutex);

  if(sp->opus != NULL){
    opus_decoder_destroy(sp->opus);
    sp->opus = NULL;
  }
  struct packet *pkt_next;
  for(struct packet *pkt = sp->queue; pkt != NULL; pkt = pkt_next){
    pkt_next = pkt->next;
    free(pkt);
  }

  // Remove from linked list
  if(sp->next != NULL)
    sp->next->prev = sp->prev;
  if(sp->prev != NULL)
    sp->prev->next = sp->next;
  else
    Audio = sp->next;
  free(sp);
  return 0;
}
void closedown(int s){
  while(Audio != NULL)
    close_session(Audio);

  Pa_Terminate();
  endwin();

  exit(0);
}

// Portaudio callback - transfer data (if any) to provided buffer
static int pa_callback(const void *inputBuffer, void *outputBuffer,
		       unsigned long framesPerBuffer,
		       const PaStreamCallbackTimeInfo* timeInfo,
		       PaStreamCallbackFlags statusFlags,
		       void *userData){
  if(outputBuffer == NULL)
    return paAbort; // can this happen??
  
  float *out = (float *)outputBuffer;

  memset(outputBuffer,0,sizeof(float)*framesPerBuffer* NCHAN);
  // Walk through each decoder control block and add its decoded audio
  struct audio *sp;
  for(sp=Audio;sp != NULL && !sp->pause; sp=sp->next){
    int sdelay = submod(sp->wptr,addmod(sp->rptr,sp->offset)) / NCHAN; // samples @ 48 kHz
    if(sdelay < -96000){
      // Writer has fallen more than 2 seconds behind, pause
      sp->pause = 1;
      continue;
    }
    int index = addmod(sp->rptr,sp->offset);
    for(int n=0;n < NCHAN*framesPerBuffer; n++){
      assert(index >= 0 && index < BUFFERSIZE);
      out[n] += sp->output_buffer[index];
      index = addmod(index,1);
    }
    sp->rptr = addmod(sp->rptr,NCHAN*framesPerBuffer);
  }
  return paContinue;
}

void *opus_task(void *arg){
  struct audio *sp = (struct audio *)arg;
  assert(sp != NULL);

  pthread_setname("opusdec");

  int error;
  sp->opus = opus_decoder_create(Samprate,NCHAN,&error);
  sp->rptr = sp->wptr = 0;
  sp->offset = 0;
  sp->pause = 0;

#define SLOP (.01) // Allow 10 ms for processing
#define RESET_DELAY (2.0) // If > 2 seconds behind, reset playout delay

  while(1){
    struct packet *pkt;

    // Peek at the first packet on the queue
    pthread_mutex_lock(&sp->qmutex);    
    pkt = sp->queue;
    pthread_mutex_unlock(&sp->qmutex);    

    if(pkt == NULL || pkt->rtp.seq != sp->eseq){
      // No packets waiting, or first packet isn't the next we expect,
      // so sleep as long as we can so packets can arrive and get sorted
      // How much time do we have until the upcall catches up with us?
      int sdelay = submod(sp->wptr,addmod(sp->rptr,sp->offset)) / NCHAN; // samples @ 48 kHz
      double qdelay = (double) sdelay / Samprate; // Seconds

      if(qdelay > SLOP){
	// We have some time to wait for packets to arrive and be resequenced
	struct timespec ts;
	ts.tv_sec = (int)(qdelay - SLOP);
	ts.tv_nsec = (int)(fmod(qdelay-SLOP,1.0) * 1e9);
	nanosleep(&ts,&ts);
      }
    }
    // Wait for packet
    pthread_mutex_lock(&sp->qmutex);
    while(sp->queue == NULL)
      pthread_cond_wait(&sp->qcond,&sp->qmutex);
    pkt = sp->queue;
    sp->queue = pkt->next;
    pkt->next = NULL;
    pthread_mutex_unlock(&sp->qmutex);

    sp->channels = opus_packet_get_nb_channels((unsigned char *)pkt->data);
    sp->opus_bandwidth = opus_packet_get_bandwidth((unsigned char *)pkt->data);
    sp->opus_frame_size = opus_packet_get_nb_samples((unsigned char *)pkt->data,pkt->len,Samprate);

    int tjump = (signed int)(pkt->rtp.timestamp - sp->etimestamp);
    int sjump = (signed short)(pkt->rtp.seq - sp->eseq);
    if(sjump < 0 || tjump < 0){
      sp->dupes++;
      free(pkt);
      continue;      // Discard old duplicate
    }
    if(sjump)
      sp->drops += sjump;

    if(tjump > 96000){
      // Loss of 2 seconds or more, reset
      sp->wptr = sp->rptr = sp->offset = 0;
      tjump = 0;
      sp->pause = 0;
    }

    // Find place to write this one
    int start_wptr = sp->wptr;
    sp->wptr = addmod(sp->wptr,tjump*NCHAN);

    // Decode frame
    float tbuffer[16384]; // Biggest possible decode? should check this
    int size = opus_decode_float(sp->opus,pkt->data,pkt->len,tbuffer,8192,0);
    for(int i=0;i<NCHAN*size;i++){
      assert(sp->wptr >= 0 && sp->wptr < BUFFERSIZE);
      sp->output_buffer[sp->wptr] = tbuffer[i] * sp->gain;
      sp->wptr = addmod(sp->wptr,1);
    }
    sp->etimestamp = pkt->rtp.timestamp + size;
    sp->eseq = pkt->rtp.seq + 1;
    free(pkt);

    int delay = submod(sp->wptr,addmod(sp->rptr,sp->offset));
    if(delay < 0)
      sp->lates++;

    sp->pause = 0;
    // Wipe old region of buffer to prevent delayed playbacks if we're late
    int write_len = submod(sp->wptr,start_wptr);

    assert(write_len >= 0 && write_len < BUFFERSIZE/2);

    start_wptr = addmod(start_wptr,BUFFERSIZE/2);

    while(write_len--){
      assert(start_wptr >= 0 && start_wptr < BUFFERSIZE);
      sp->output_buffer[start_wptr] = 0;
      start_wptr = addmod(start_wptr,1);
    }

  }
}
