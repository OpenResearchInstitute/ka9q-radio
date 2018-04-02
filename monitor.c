// $Id: monitor.c,v 1.43 2018/04/02 10:26:20 karn Exp karn $
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
#define USE_COND 1            // Use pthread condition to signal from callback (portaudio deprecates this)
#define SAMPRATE 48000        // Too hard to handle other sample rates right now
#define MAX_MCAST 20          // Maximum number of multicast addresses
#define PKTSIZE 16384         // Maximum bytes per RTP packet - must be bigger than Ethernet MTU
#define NCHAN 2               // 2 channels (stereo)
#define SAMPPCALLBACK (SAMPRATE/100)     // 10 ms @ 48 kHz
#define BUFFERSIZE (1<<19)    // about 10.92 sec at 48 kHz stereo - must be power of 2!!
#define PAUSETHRESH (4*SAMPRATE)     // Pause after this many seconds of starvation

char *Mcast_address_text[MAX_MCAST]; // Multicast address(es) we're listening to
char Audiodev[256];           // Name of audio device; empty means portaudio's default
int Update_interval = 100;    // Default time in ms between display updates
int List_audio;               // List audio output devices and exit
int Verbose;                  // Verbosity flag (currently unused)
int Quiet;                    // Disable curses
int Nfds;                     // Number of streams
struct session *Session;      // Link to head of session structure chain
PaStream *Pa_Stream;          // Portaudio stream handle
int inDevNum;                 // Portaudio's audio output device index

// The portaudio callback continuously plays out this single buffer, into which multiple streams sum their audio
// and the callback zeroes out each sample as played
float const SCALE = 1./SHRT_MAX;
WINDOW *Mainscr;

#if USE_COND
pthread_cond_t Upcall_cond;
pthread_mutex_t Buffer_mutex;
#endif

struct timeval Start_unix_time;
PaTime Start_pa_time;

// Incoming RTP packets
struct packet {
  struct packet *next;
  struct rtp_header rtp;
  unsigned char data[PKTSIZE];
  int len;
};

struct session {
  struct session *prev;       // Linked list pointers
  struct session *next; 
  uint32_t ssrc;            // RTP Sending Source ID
  uint16_t eseq;            // Next expected RTP sequence number
  int basetime;        // RTP timestamp corresponding to approx current playback time
  int etimestamp;           // Next expected RTP timestamp
  int type;                 // RTP type (10,11,20,111)
  int reseq_count;
  
  struct packet *queue;     // Incoming RTP packets

  float gain;               // Gain for this channel; 1 -> 0 db (nominal)

  struct sockaddr sender;
  char *dest;
  char addr[NI_MAXHOST];    // RTP Sender IP address
  char port[NI_MAXSERV];    // RTP Sender source port
  OpusDecoder *opus;        // Opus codec decoder handle, if needed
  int channels;             // Channels (1 or 2)
  int frame_size;           // Samples in a frame
  int opus_bandwidth;       // Opus stream audio bandwidth

  pthread_t task;           // Thread processing packets and running decoder
  pthread_mutex_t qmutex;   // Mutex protecting packet queue
  pthread_cond_t qcond;     // Condition variable for arrival of new packet

  unsigned long packets;    // RTP packets for this session
  unsigned long drops;      // Apparent rtp packet drops
  unsigned long empties;    // RTP but no data
  unsigned long dupes;      // Duplicate sequence numbers
  unsigned long lates;      // Callback count of underruns (stereo samples) replaced with silence
  unsigned long erasures;   // Erasure gaps

  float output_buffer[BUFFERSIZE][NCHAN]; // Decoded audio output, written by processing thread and read by PA callback
  int wptr;                        // Write pointer into output_buffer
  int rptr;                        // Read pointer into output buffer
  int playout;                      // Playout delay
  int running;                     // Callback is reading our output buffer and advancing rptr
};
struct session *Current;

unsigned long long Samples;
unsigned long long Callbacks;


void closedown(int);
void *display(void *);
struct session *lookup_session(const struct sockaddr *,uint32_t);
struct session *create_session(struct sockaddr const *,uint32_t);
int close_session(struct session *);
static int pa_callback(const void *,void *,unsigned long,const PaStreamCallbackTimeInfo*,PaStreamCallbackFlags,void *);
void *decode_task(void *x);

// Convert unsigned number modulo buffersize to a signed 2's complement
static inline int signmod(unsigned int const a){
  int y = a & (BUFFERSIZE-1);
  
  if(y >= BUFFERSIZE/2)
    y -= BUFFERSIZE;
  assert(y >= -BUFFERSIZE/2 && y < BUFFERSIZE/2);
  return y;
}



int main(int argc,char * const argv[]){
  // Try to improve our priority, then drop root
  int prio = getpriority(PRIO_PROCESS,0);
  prio = setpriority(PRIO_PROCESS,0,prio - 15);
  seteuid(getuid());

  setlocale(LC_ALL,getenv("LANG"));

  int c;
  while((c = getopt(argc,argv,"R:S:I:vLqu:")) != EOF){
    switch(c){
    case 'L':
      List_audio++;
      break;
    case 'R':
      strncpy(Audiodev,optarg,sizeof(Audiodev));
      break;
    case 'v':
      Verbose++;
      break;
    case 'I':
      if(Nfds == MAX_MCAST){
	fprintf(stderr,"Too many multicast addresses; max %d\n",MAX_MCAST);
      } else 
	Mcast_address_text[Nfds++] = optarg;
      break;
    case 'q': // No ncurses
      Quiet++;
      break;
    case 'u':
      Update_interval = strtol(optarg,NULL,0);
      break;
    default:
      fprintf(stderr,"Usage: %s [-v] [-q] [-L] [-R audio device] -I mcast_address [-I mcast_address]\n",argv[0]);
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

  // Set up multicast input, create mask for select()
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
  // Create portaudio stream.
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
		    SAMPRATE,
		    //paFramesPerBufferUnspecified, // seems to be 31 on OSX
		    SAMPPCALLBACK,
		    0,
		    pa_callback,
		    NULL);

  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));      
    exit(1);
  }

#if USE_COND
  pthread_cond_init(&Upcall_cond,NULL);
  pthread_mutex_init(&Buffer_mutex,NULL);
#endif

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);
  if(!Quiet){
    pthread_t display_task;
    pthread_create(&display_task,NULL,display,NULL);
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

  // Do this at the last minute at startup since the upcall will come quickly
  r = Pa_StartStream(Pa_Stream);
  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));
    exit(1);
  }
  Start_pa_time = Pa_GetStreamTime(Pa_Stream);
  gettimeofday(&Start_unix_time,NULL);

  struct packet *pkt = NULL;

  // Main loop begins here
  while(1){
    // Wait for traffic to arrive
    fd_set fdset = fdset_template;
    int s = select(max_fd+1,& fdset,NULL,NULL,NULL);
    if(s < 0 && errno != EAGAIN && errno != EINTR)
      break;
    if(s == 0)
      continue; // Nothing arrived; probably just an ignored signal

    for(int fd_index = 0;fd_index < Nfds;fd_index++){
      if(input_fd[fd_index] == -1)
	continue;
      if(!FD_ISSET(input_fd[fd_index],&fdset))
	continue;

      // Need a new packet buffer?
      if(!pkt)
	pkt = malloc(sizeof(*pkt));
      pkt->next = NULL; // just to be safe

      // message points to iovec[]
      iovec[0].iov_base = &pkt->rtp;
      iovec[0].iov_len = sizeof(pkt->rtp);
      iovec[1].iov_base = &pkt->data;
      iovec[1].iov_len = sizeof(pkt->data);

      int size = recvmsg(input_fd[fd_index],&message,0);
      if(size == -1){
	if(errno != EINTR){ // Happens routinely, e.g., when window resized
	  perror("recvmsg");
 	  usleep(1000);
	}
	continue;
      }
      if(size <= sizeof(pkt->rtp))
	continue; // Must be big enough for RTP header and at least some data

      int type = pkt->rtp.mpt & ~RTP_MARKER;
      if(type != PCM_STEREO_PT
	 && type != OPUS_PT
	 && type != 20           // Old fixed value for Opus, take this out eventually
	 && type != PCM_MONO_PT) // 1 byte, no need to byte swap
	continue; // Discard unknown RTP types to avoid polluting session table
      
      // Convert RTP header to host order
      pkt->rtp.ssrc = ntohl(pkt->rtp.ssrc);
      pkt->rtp.seq = ntohs(pkt->rtp.seq);
      pkt->rtp.timestamp = ntohl(pkt->rtp.timestamp);
      pkt->len = size - sizeof(pkt->rtp); // Bytes in payload
      assert(pkt->len > 0);

      // Find appropriate session; create new one if necessary
      struct session *sp = lookup_session(&sender,pkt->rtp.ssrc);
      if(!sp){
	// Not found
	if(!(sp = create_session(&sender,pkt->rtp.ssrc))){
	  fprintf(stderr,"No room!!\n");
	  continue;
	}
	getnameinfo((struct sockaddr *)&sender,sizeof(sender),sp->addr,sizeof(sp->addr),
		    //		    sp->port,sizeof(sp->port),NI_NOFQDN|NI_DGRAM|NI_NUMERICHOST);
		    sp->port,sizeof(sp->port),NI_NOFQDN|NI_DGRAM);
	sp->dest = Mcast_address_text[fd_index];
	sp->eseq = pkt->rtp.seq; // Needed since we sort the queue by sequence number

	pthread_mutex_init(&sp->qmutex,NULL);
	pthread_cond_init(&sp->qcond,NULL);
	if(pthread_create(&sp->task,NULL,decode_task,sp) == -1){
	  perror("pthread_create");
	  close_session(sp);
	  continue;
	}
      }

      // Insert onto queue sorted by sequence number, wake up thread
      struct packet *q_prev = NULL;
      struct packet *qe = NULL;
      pthread_mutex_lock(&sp->qmutex);
      for(qe = sp->queue;qe; q_prev = qe,qe = qe->next){
	if(pkt->rtp.seq < qe->rtp.seq)
	  break;
      }
      pkt->next = qe;
      if(q_prev)
	q_prev->next = pkt;
      else
	sp->queue = pkt; // Front of list
      pkt = NULL;        // force new packet to be allocated
      // wake up decoder thread (used first time only, after that it's timer polled)
      pthread_cond_signal(&sp->qcond);
      pthread_mutex_unlock(&sp->qmutex);
    }      
  }
  echo();
  nocbreak();
  endwin();
  exit(0);
}


// Portaudio callback - transfer data (if any) to provided buffer
static int pa_callback(const void *inputBuffer, void *outputBuffer,
		       unsigned long framesPerBuffer,
		       const PaStreamCallbackTimeInfo* timeInfo,
		       PaStreamCallbackFlags statusFlags,
		       void *userData){
  if(!outputBuffer)
    return paAbort; // can this happen??
  
  assert(framesPerBuffer < BUFFERSIZE/2); // Make sure ring buffer is big enough

  float (*out)[NCHAN] = outputBuffer;
  
  memset(out,0,sizeof(*out) * framesPerBuffer);
  // Walk through each decoder control block and add its decoded audio into output
  struct session *sp;
  for(sp=Session; sp; sp=sp->next){
    if(!sp->running)
      continue;

    // Use temp for index so rptr is updated in steps
    // This is important so the sleep in the decoder sleeps the entire interval
    int index = sp->rptr;
    for(int n=0;n < framesPerBuffer; n++){
      for(int j = 0; j < NCHAN; j++){
	out[n][j] += sp->output_buffer[index][j];
	sp->output_buffer[index][j] = 0; // Burn after reading
      }
      index++;
      index &= (BUFFERSIZE-1);
    }
    sp->rptr = index;
  }
  Samples += framesPerBuffer;
  Callbacks++;
#if USE_COND
  pthread_mutex_lock(&Buffer_mutex);
  pthread_cond_broadcast(&Upcall_cond);
  pthread_mutex_unlock(&Buffer_mutex);  
#endif
  return paContinue;
}

void decode_task_cleanup(void *arg){
  struct session *sp = (struct session *)arg;
  assert(sp);

  pthread_mutex_destroy(&sp->qmutex);

  if(sp->opus){
    opus_decoder_destroy(sp->opus);
    sp->opus = NULL;
  }
  struct packet *pkt_next;
  for(struct packet *pkt = sp->queue; pkt; pkt = pkt_next){
    pkt_next = pkt->next;
    free(pkt);
  }
}

// Thread to decode incoming RTP packets for each stream
void *decode_task(void *arg){
  struct session *sp = (struct session *)arg;
  assert(sp);

  pthread_setname("decode");
  pthread_cleanup_push(decode_task_cleanup,arg);

  sp->gain = 1;    // 0 dB by default
  sp->playout = 0;  // Bad things can happen if this is negative during the first packet, as sp->frame_size may not get set
  sp->wptr = SAMPPCALLBACK; // Ahead of the D/A by one callback block, since it can happen at any time

  struct packet *pkt = NULL;

  // Wait for the first packet to appear on queue, which should happen shortly
  pthread_mutex_lock(&sp->qmutex);
  while(!sp->queue)
    pthread_cond_wait(&sp->qcond,&sp->qmutex);
  pkt = sp->queue;
  sp->queue = pkt->next;
  pkt->next = NULL;
  pthread_mutex_unlock(&sp->qmutex);
  sp->basetime = sp->etimestamp = pkt->rtp.timestamp; // note: sp->eseq is already initialized

  int lastmarker;

  // Main loop; run until canceled
  while(1){

    if(pkt){
      sp->packets++;

      int seq_offset = (signed short)(pkt->rtp.seq - sp->eseq);
      if(seq_offset < -5 || seq_offset > 50){
	if(++sp->reseq_count >= 3){
	  //resynch
	  sp->eseq = pkt->rtp.seq;
	  sp->basetime = sp->etimestamp = pkt->rtp.timestamp;
	  sp->playout = 0;
	} else {
	  // Discard wildly different sequence numbers
	  sp->dupes++;
	  free(pkt);  pkt = NULL;
	  goto nextpacket;
	}
      }
      sp->reseq_count = 0;
      sp->type = pkt->rtp.mpt & ~RTP_MARKER;
      int marker = pkt->rtp.mpt & RTP_MARKER;
      if(seq_offset > 0)
	sp->drops += seq_offset;

      sp->eseq = pkt->rtp.seq + 1;

      int time_offset = (int)(pkt->rtp.timestamp - sp->basetime);
      
      if(!seq_offset && (marker || (int)(pkt->rtp.timestamp - sp->etimestamp) > 0)){
	// In sequence, beginning of talk spurt after silence suppression
	// Good time to shrink the playout buffer
	int silence = (int)(pkt->rtp.timestamp - sp->etimestamp);
	lastmarker = pkt->rtp.timestamp;
	sp->playout = max(-time_offset,sp->playout - silence);
	//	printf("marker, base %d time offset %d silence %d new playout %d\n",sp->basetime,
	//	       time_offset,silence,sp->playout);
      } else if(time_offset < -sp->playout){
	// This shouldn't happen if we just shrank the playout buffer...
	// Late packet during talk spurt, drop as late
	//	printf("late: base %d since last marker %d time offset %d playout %d\n",
	//	       sp->basetime,(int)(pkt->rtp.timestamp - lastmarker),time_offset,sp->playout);
	sp->lates++;
	sp->playout += sp->frame_size; // increase playout in quanta of frames
	sp->etimestamp = pkt->rtp.timestamp + sp->frame_size;
	free(pkt); pkt = NULL;
	goto nextpacket;
      }
      // Decode frame, write into output buffer
      //    printf("Processing at %d\n",pkt->rtp.timestamp);
      int wp = (time_offset + sp->playout + sp->wptr) & (BUFFERSIZE-1);
      switch(sp->type){
      case PCM_STEREO_PT:
	sp->channels = 2;
	sp->frame_size = pkt->len / 4; // Number of stereo samples
	signed short (*data_stereo_ints)[NCHAN] = (signed short (*)[NCHAN])pkt->data;
	for(int i=0; i < sp->frame_size; i++){
	  for(int j=0; j < NCHAN; j++)
	    sp->output_buffer[wp][j] = SCALE * ntohs(data_stereo_ints[i][j]) * sp->gain;
	  wp++; wp &= (BUFFERSIZE-1);
	}
	break;
      case PCM_MONO_PT:
	sp->channels = 1;
	sp->frame_size = pkt->len / 2; // Number of mono samples      
	signed short *data_ints = (signed short *)pkt->data;
	for(int i=0; i < sp->frame_size; i++){
	  sp->output_buffer[wp][0] = SCALE * ntohs(data_ints[i]) * sp->gain;
	  for(int j=1; j < NCHAN; j++)
	    sp->output_buffer[wp][j] = sp->output_buffer[wp][0];
	  wp++; wp &= (BUFFERSIZE-1);
	}
	break;
      case OPUS_PT:
      case 20:
	if(!sp->opus){
	  int error;
	  sp->opus = opus_decoder_create(SAMPRATE,NCHAN,&error);
	}
	sp->channels = opus_packet_get_nb_channels(pkt->data);
	sp->opus_bandwidth = opus_packet_get_bandwidth(pkt->data);
	sp->frame_size = opus_packet_get_nb_samples(pkt->data,pkt->len,SAMPRATE);
	{
	  float bounce[sp->frame_size][NCHAN];
	  int samples = opus_decode_float(sp->opus,pkt->data,pkt->len,&bounce[0][0],sp->frame_size,0);	
	  assert(samples == sp->frame_size);
	  if(samples > 0){	// check for error of some kind
	    for(int i=0; i<samples; i++){
	      for(int j=0; j < NCHAN; j++)
		sp->output_buffer[wp][j] = bounce[i][j] * sp->gain;
	      wp++; wp &= (BUFFERSIZE-1);
	    }
	  }
	}
	break;
      }
      sp->etimestamp = pkt->rtp.timestamp + sp->frame_size;
      free(pkt); pkt = NULL;
      sp->running = 1; // Start at last moment
    } else if (sp->frame_size){
      // pkt == NULL; Nothing arrived in time, but only if we have earlier packets to tell us the frame size
      sp->erasures++;
      //      printf("erasure at etimestamp %d\n",sp->etimestamp);
      // We didn't get what we wanted in time; insert erasures
      int wp = (sp->playout + sp->wptr) & (BUFFERSIZE-1);
      switch(sp->type){
#if 0 // not really needed for PCM since it'll already be zeroed 
      case PCM_STEREO_PT:
      case PCM_MONO_PT:
	if(wp + sp->frame_size <= BUFFERSIZE)
	  memset(sp->output_buffer[wp],0,sp->frame_size * sizeof(sp->output_buffer[wp]));
	else {
	  memset(sp->output_buffer[wp],0,(BUFFERSIZE - wp) * sizeof(sp->output_buffer[wp]));	  
	  memset(sp->output_buffer[0],0,(wp + sp->frame_size - BUFFERSIZE) * sizeof(sp->output_buffer[wp]));	  
	}
	break;
#endif
      case OPUS_PT:
      case 20:
	{
	  // Opus erasure handling - should also support FEC through look-ahead
	  float bounce[sp->frame_size][NCHAN];
	  int samples = opus_decode_float(sp->opus,NULL,0,&bounce[0][0],sp->frame_size,0);	
	  assert(samples == sp->frame_size);
	  if(samples > 0){
	    for(int i=0; i<samples; i++){
	      for(int j=0; j < NCHAN; j++)
		sp->output_buffer[wp][j] = bounce[i][j] * sp->gain;
	      wp++; wp &= (BUFFERSIZE-1);
	    }
	  }
	}
	break;
      }
      sp->running = 1; // Start at last moment
    }
    // We must always go through here on every loop iteration to advance the time by one frame
    // i.e., avoid continue statements above
  nextpacket: ;
    assert(!pkt); // any packet must be consumed by this point
    while(1){
      // Poll the queue
      pthread_mutex_lock(&sp->qmutex);
      if(sp->queue){
	// Grab it
	pkt = sp->queue;
	sp->queue = pkt->next;
	pkt->next = NULL;
      }
      pthread_mutex_unlock(&sp->qmutex);    
      if(pkt)
	break;

      assert(sp->frame_size != 0); // Should have been set by now

      // Nothing on queue; wait for the D/A to catch up and look again
      sp->wptr += sp->frame_size;
      sp->wptr &= (BUFFERSIZE-1);

#if USE_COND
      pthread_mutex_lock(&Buffer_mutex);
      while(signmod(sp->wptr - sp->rptr) > SAMPPCALLBACK)
	pthread_cond_wait(&Upcall_cond,&Buffer_mutex);
      pthread_mutex_unlock(&Buffer_mutex);      
#else // use timer
      while(signmod(sp->wptr - sp->rptr) > SAMPPCALLBACK)
	usleep(1000);  // Nothing else to do; wait for the D/A to catch up with us
#endif
      sp->basetime += sp->frame_size;
    }

  }
  pthread_cleanup_pop(1);
  return NULL;
}
// Use ncurses to display streams
void *display(void *arg){

  initscr();
  keypad(stdscr,TRUE);
  timeout(Update_interval);
  cbreak();
  noecho();
  
  Mainscr = stdscr;

  while(1){

    int row = 2;
    wmove(Mainscr,row,0);
    wclrtobot(Mainscr);
    mvwprintw(Mainscr,row++,0,"Type        ch BW Gain      SSRC     Packets   Dupe   Drop      Late  Erasures Queue Source                Dest");

    for(struct session *sp = Session; sp; sp = sp->next){
      int bw = 0; // Audio bandwidth (not bitrate) in kHz
      char *type,typebuf[30];
      switch(sp->type){
      case PCM_STEREO_PT:
      case PCM_MONO_PT:
	type = "PCM";
	bw = SAMPRATE / 2000;
	break;
      case 20: // for temporary backward compatibility
      case OPUS_PT:
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
	snprintf(typebuf,sizeof(typebuf),"Opus %.1lf ms",1000.*sp->frame_size/SAMPRATE);
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
      if(sp->running)
	wattr_on(Mainscr,A_BOLD,NULL); // Embolden active streams

      char source_text[256];
      snprintf(source_text,sizeof(source_text),"%s:%s",sp->addr,sp->port);
      if(sp == Current)
	wattr_on(Mainscr,A_UNDERLINE,NULL);
      mvwprintw(Mainscr,row++,0,"%-12s%2d%3d%+5.0lf%10x%'12lu%7lu%7lu%'10lu%'10lu%6.3lf %-22s%-16s",
		type,
		sp->channels,
		bw,
		20*log10(sp->gain),
		sp->ssrc,
		sp->packets,
		sp->dupes,
		sp->drops,
		sp->lates,
		sp->erasures,
		(double)signmod(sp->etimestamp - sp->basetime)/SAMPRATE,
		source_text,
		sp->dest);
      wattr_off(Mainscr,A_BOLD,NULL);
      wattr_off(Mainscr,A_UNDERLINE,NULL);
      if(!Current)
	Current = sp;
    }
    row++;
    mvwprintw(Mainscr,row++,0,"TAB: select next stream; d: delete stream; up/down arrow: volume +/-1 dB; left/right arrow: playout delay -/+ 10 ms\n");

    // Measure skew between sampling clock and UNIX real time (hopefully NTP synched)
    struct timeval tv;
    gettimeofday(&tv,NULL);
    double unix_seconds = tv.tv_sec - Start_unix_time.tv_sec + 1e-6*(tv.tv_usec - Start_unix_time.tv_usec);
    double pa_seconds = Pa_GetStreamTime(Pa_Stream) - Start_pa_time;
    mvwprintw(Mainscr,row++,0,"D/A clock error: %lf ppm\n",1e6 * (pa_seconds / unix_seconds - 1));

    move(row,0);
    clrtobot();
    mvwprintw(Mainscr,0,0,"KA9Q Multicast Audio Monitor:");
    for(int i=0;i<Nfds;i++)
      wprintw(Mainscr," %s",Mcast_address_text[i]);
    wprintw(Mainscr,"\n");
    wnoutrefresh(Mainscr);
    doupdate();
    if(Current){ // process commands only if there's something to act on
      int c = getch(); // Pauses here
      switch(c){
      case EOF:
	break;
      case KEY_NPAGE:
      case '\t':
	if(Current->next)
	  Current = Current->next;
	else if(Session)
	  Current = Session; // Wrap around to top
	break;
      case KEY_BTAB:
      case KEY_PPAGE:
	if(Current->prev)
	  Current = Current->prev;
	break;
      case KEY_UP:
	Current->gain *= 1.122018454; // 1 dB
	break;
      case KEY_DOWN:
	Current->gain /= 1.122018454; // 1 dB
	break;
      case KEY_RIGHT:
	Current->playout += 480; // 10 ms
	break;
      case KEY_LEFT:
	Current->playout = max(0,Current->playout - 480);
	break;
      case 'd':
	{
	  struct session *next = Current->next;
	  close_session(Current);
	  Current = next;
	}
	break;
      case '\f':  // Screen repaint (formfeed, aka control-L)
	clearok(curscr,TRUE);
	break;
      default:
	break;
      }
    }
  }
  return NULL;
}

struct session *lookup_session(const struct sockaddr *sender,const uint32_t ssrc){
  struct session *sp;
  for(sp = Session; sp; sp = sp->next){
    if(sp->ssrc == ssrc && memcmp(&sp->sender,sender,sizeof(*sender)) == 0){
      // Found it
      return sp;
    }
  }
  return NULL;
}
// Create a new session, partly initialize
struct session *create_session(struct sockaddr const *sender,uint32_t ssrc){
  struct session *sp;

  if(!(sp = calloc(1,sizeof(*sp))))
    return NULL; // Shouldn't happen on modern machines!
  
  // Initialize entry
  memcpy(&sp->sender,sender,sizeof(struct sockaddr));
  sp->ssrc = ssrc;
  // Put at head of list
  sp->next = Session;
  if(sp->next)
    sp->next->prev = sp;
  Session = sp;
  return sp;
}

int close_session(struct session *sp){
  if(!sp)
    return -1;
  
  sp->running = 0;
  
  // Remove from linked list
  if(sp->next)
    sp->next->prev = sp->prev;
  if(sp->prev)
    sp->prev->next = sp->next;
  else
    Session = sp->next;

  pthread_cancel(sp->task);
  pthread_join(sp->task,NULL);

  free(sp);
  return 0;
}
void closedown(int s){
  while(Session)
    close_session(Session);

  Pa_Terminate();
  endwin();

  exit(0);
}
