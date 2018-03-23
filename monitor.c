// $Id: monitor.c,v 1.36 2018/03/05 04:36:06 karn Exp karn $
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
const int Slop = 480;         // Playout delay in samples to add after an underrun
#define Aud_bufsize (1<<18)   // make this a power of 2 for efficiency
const int Bufsize = 8192;     // Maximum samples/words per RTP packet - must be bigger than Ethernet MTU
const int Samprate = 48000;   // Too hard to handle other sample rates right now
int List_audio;               // List audio output devices and exit
int Verbose;                  // Verbosity flag (currently unused)
int Update_interval = 100;    // Time in ms between display updates
int Age_limit = 1000;          // Delete after 100 sec
int Highlight_limit = 10;     // Dim after 1 second
#define NCHAN 2               // 2 channels (stereo)



int Nfds;                    // Number of streams
struct audio *Audio;          // Link to head of session structure chain
PaStream *Pa_Stream;          // Portaudio stream handle
int inDevNum;                 // Portaudio's audio output device index

// The portaudio callback continuously plays out this single buffer, into which multiple streams sum their audio
// and the callback zeroes out each sample as played
float Audio_buffer[Aud_bufsize]; // Audio playout buffer
float const SCALE = 1./SHRT_MAX;
WINDOW *Mainscr;

pthread_cond_t Upcall_cond;
pthread_mutex_t Upcall_mutex;
pthread_mutex_t Buffer_mutex;


struct queue {
  struct queue *next;
  struct rtp_header rtp;
  unsigned char *data;
  int len;
};



struct audio {
  struct audio *prev;       // Linked list pointers
  struct audio *next; 
  uint32_t ssrc;            // RTP Sending Source ID
  uint16_t eseq;                 // Next expected RTP sequence number
  int etime;                // Next expected RTP timestamp
  int type;                 // RTP type (10,11,20)
  
  struct queue *queue;

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



  unsigned long packets;    // RTP packets for this session
  unsigned long drops;      // Apparent rtp packet drops
  unsigned long invalids;   // Unknown RTP type
  unsigned long empties;    // RTP but no data
  unsigned long dupes;      // Duplicate or old serial numbers
  int age;                  // Display cycles since last active
  unsigned long underruns;  // Callback count of underruns (stereo samples) replaced with silence
  int hw_delay;             // Estimated playout delay in samples
};
struct audio *Current = NULL;

void closedown(int);
void display();
struct audio *lookup_session(const struct sockaddr *,uint32_t);
struct audio *create_session(struct sockaddr const *,uint32_t);
int close_session(struct audio *);
static int pa_callback(const void *,void *,unsigned long,const PaStreamCallbackTimeInfo*,PaStreamCallbackFlags,void *);

void *opus_task(void *x);


int main(int argc,char * const argv[]){
  // Try to improve our priority
  int prio = getpriority(PRIO_PROCESS,0);
  prio = setpriority(PRIO_PROCESS,0,prio - 15);

  // Drop root if we have it
  seteuid(getuid());

  setlocale(LC_ALL,getenv("LANG"));

  int c;
  while((c = getopt(argc,argv,"a:S:I:vLE:A:")) != EOF){
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
    default:
      fprintf(stderr,"Usage: %s [-v] [-I mcast_address]\n",argv[0]);
      exit(1);
    }
  }
  if(Nfds == 0){
    fprintf(stderr,"At least one -I option required\n");
    exit(1);
  }


  pthread_cond_init(&Upcall_cond,NULL);
  pthread_mutex_init(&Upcall_mutex,NULL);
  pthread_mutex_init(&Buffer_mutex,NULL);


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
  struct iovec iovec[2];
  struct rtp_header rtp;
  //  int16_t data[Bufsize];
  signed short data[Bufsize];
  
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

  // Set up multicast input
  fd_set fdset_template; // Mask for select()
  FD_ZERO(&fdset_template);
  int max_fd = 2;        // Highest number fd for select()
  int input_fd[Nfds];    // Multicast receive sockets

  for(int i=0;i<Nfds;i++){
    input_fd[i] = setup_mcast(Mcast_address_text[i],0);
    if(input_fd[i] == -1){
      fprintf(stderr,"Can't set up input %s\n",Mcast_address_text[i]);
    }
    if(input_fd[i] > max_fd)
      max_fd = input_fd[i];
    FD_SET(input_fd[i],&fdset_template);
    // Temporarily put socket in nonblocking state
    // and flush any pending packets to avoid unnecessary long buffer delays
    int flags;
    if((flags = fcntl(input_fd[i],F_GETFL)) != -1){
      flags |= O_NONBLOCK;
      if(fcntl(input_fd[i],F_SETFL,flags) == 0){
	int flushed = 0;
	while(recvmsg(input_fd[i],&message,0) >= 0)
	  flushed++;
	flags &= ~O_NONBLOCK;
	fcntl(input_fd[i],F_SETFL,flags);
      }
    }
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
  
  r = Pa_OpenStream(&Pa_Stream,NULL,&outputParameters,Samprate,
		    //		    paFramesPerBufferUnspecified,
		    960, // Match 20ms preferred Opus blocksize
		    0,pa_callback,NULL);

  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));      
    exit(1);
  }


  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);

  initscr();
  keypad(stdscr,TRUE);
  timeout(0); // Nonblocking
  cbreak();
  noecho();

  Mainscr = stdscr;

  struct timeval last_update_time;
  gettimeofday(&last_update_time,NULL);

  // Do this last since the upcall will come quickly
  r = Pa_StartStream(Pa_Stream);
  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));
    exit(1);
  }


  while(1){
    struct timeval tp;
    
    gettimeofday(&tp,NULL);
    // Time since last update in milliseconds
    int interval = (tp.tv_sec - last_update_time.tv_sec)*1000 + (tp.tv_usec - last_update_time.tv_usec)/1000.;
    if(interval >= Update_interval){
      last_update_time = tp;
      display();
    }
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000 * Update_interval;

    fd_set fdset;
    fdset = fdset_template;
    //    FD_COPY(&fdset_template,&fdset); // not on Linux; simple assign seems to work
    int s = select(max_fd+1,&fdset,NULL,NULL,&timeout);
    if(s < 0 && errno != EAGAIN && errno != EINTR)
      break;
    if(s == 0)
      continue;

    for(int fd_index = 0;fd_index < Nfds;fd_index++){
      if(!FD_ISSET(input_fd[fd_index],&fdset))
      continue;

      int size = recvmsg(input_fd[fd_index],&message,0);
      if(size == -1){
	if(errno != EINTR){ // Happens routinely
	  perror("recvmsg");
	  usleep(1000);
	}
	continue;
      }
      if(size < sizeof(rtp)){
	usleep(500); // Avoid tight loop
	continue; // Too small to be valid RTP
      }
      // To host order
      rtp.ssrc = ntohl(rtp.ssrc);
      rtp.seq = ntohs(rtp.seq);
      rtp.timestamp = ntohl(rtp.timestamp);
      size -= sizeof(rtp); // Bytes in payload
      
      if(rtp.mpt != 10 && rtp.mpt != 20 && rtp.mpt != 11) // 1 byte, no need to byte swap
	continue; // Discard unknown RTP types to avoid polluting session table
      
      struct audio *sp = lookup_session(&sender,rtp.ssrc);
      if(sp == NULL){
	// Not found
	if((sp = create_session(&sender,rtp.ssrc)) == NULL){
	  fprintf(stderr,"No room!!\n");
	  continue;
	}
	getnameinfo((struct sockaddr *)&sender,sizeof(sender),sp->addr,sizeof(sp->addr),
		    //		    sp->port,sizeof(sp->port),NI_NOFQDN|NI_DGRAM|NI_NUMERICHOST);
		    sp->port,sizeof(sp->port),NI_NOFQDN|NI_DGRAM);
	sp->dest = Mcast_address_text[fd_index];
	sp->etime = rtp.timestamp;
	sp->eseq = rtp.seq;
	
	sp->gain = 1; // 0 dB by default
	sp->dupes = 0;
	pthread_mutex_init(&sp->qmutex,NULL);
	pthread_create(&sp->task,NULL,opus_task,sp);
      }
      sp->age = 0;
      sp->packets++;
      sp->type = rtp.mpt;
      switch(rtp.mpt){
      case 10:
	sp->channels = 2;
	break;
      case 11:
	sp->channels = 1;
	break;
      case 20:
	sp->channels = opus_packet_get_nb_channels((unsigned char *)data);
	sp->opus_bandwidth = opus_packet_get_bandwidth((unsigned char *)data);
	sp->opus_frame_size = opus_packet_get_nb_samples((unsigned char *)data,size,Samprate);
	break;
      }
      if((signed short)(rtp.seq - sp->eseq) < 0){
	sp->dupes++;
	continue;      // Discard old duplicate
      }
      struct queue *qentry = calloc(1,sizeof(*qentry));
      qentry->rtp = rtp;
      qentry->data = malloc(size);
      memcpy(qentry->data,data,size);
      qentry->len = size;
      
      // Sort onto list
      struct queue *q_prev = NULL;
      struct queue *qp = NULL;
      pthread_mutex_lock(&sp->qmutex);
      
      for(qp = sp->queue;qp != NULL; q_prev = qp,qp = qp->next){
	if(rtp.seq < qp->rtp.seq)
	  break;
      }
      qentry->next = qp;
      if(q_prev != NULL)
	q_prev->next = qentry;
      else
	sp->queue = qentry; // Front of list
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
    mvwprintw(Mainscr,row++,0,"Type        chans BW   Gain      SSRC     Packets  Dupes  Drops Underruns  Queue Source                Dest");
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

      int qlen = 0;
      pthread_mutex_lock(&sp->qmutex);
      for(struct queue *qp = sp->queue; qp != NULL; qp = qp->next)
	qlen++;
      pthread_mutex_unlock(&sp->qmutex);

      char source_text[256];
      snprintf(source_text,sizeof(source_text),"%s:%s",sp->addr,sp->port);
      mvwprintw(Mainscr,row,0,"%-15s%2d%3d%+7.0lf%10x%'12lu%7lu%7lu%'10lu%7d %-22s%-16s",
		type,
		sp->channels,
		bw,
		20*log10(sp->gain),
		sp->ssrc,
		sp->packets,
		sp->dupes,
		sp->drops,
		sp->underruns,
		qlen,
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
  struct queue *qp_next;
  for(struct queue *qp = sp->queue; qp != NULL; qp = qp_next){
    qp_next = qp->next;
    free(qp->data);
    free(qp);
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
// When buffer is empty, pad with silence

#define Output_size 5760 // Max output samples: 120 ms @ 48 kHz

float * volatile Output_buffer; // Copy of pointer to callback buffer so decoder thread can write to it

int32_t Last_frame;

static int pa_callback(const void *inputBuffer, void *outputBuffer,
		       unsigned long framesPerBuffer,
		       const PaStreamCallbackTimeInfo* timeInfo,
		       PaStreamCallbackFlags statusFlags,
		       void *userData){
  if(outputBuffer == NULL)
    return paAbort; // can this happen??
  
  Output_buffer = outputBuffer;
  memset(Output_buffer,0,sizeof(float)*framesPerBuffer* NCHAN); // where do we get #channels?

  pthread_mutex_lock(&Upcall_mutex);
  Last_frame += framesPerBuffer;
  pthread_cond_broadcast(&Upcall_cond);
  pthread_mutex_unlock(&Upcall_mutex);  

  // Allow time for decoder threads to add their stuff to the buffer
  // 750 == 3/4 of frame interval; 500 = 1/2 of frame interval, etc
  Pa_Sleep(750*framesPerBuffer/(Samprate)); // Compute this better
  return paContinue;
}




void *opus_task(void *arg){
  struct audio *sp = (struct audio *)arg;
  assert(sp != NULL);

  pthread_setname("opusdec");

  int32_t frame_no = Last_frame;

  float residual[NCHAN*Output_size];
  int residual_frames = 0;

  int error;
  sp->opus = opus_decoder_create(Samprate,NCHAN,&error);


  while(1){
    // Wait for callback to wake us up
    pthread_mutex_lock(&Upcall_mutex);
    while(frame_no == Last_frame)
      pthread_cond_wait(&Upcall_cond,&Upcall_mutex);

    int frames_needed = Last_frame - frame_no; // #frames needed
    frame_no = Last_frame;
    pthread_mutex_unlock(&Upcall_mutex);

    int opp = 0;
    // All this must complete before the Pa_Sleep() in the callback finishes
    while(frames_needed > 0){
      if(residual_frames){
	// Leftover frames from last decoder call
	int frame_count = min(frames_needed,residual_frames);
	
	// Lock access to upcall buffer so multiple threads don't do this all at once
	pthread_mutex_lock(&Buffer_mutex);
	for(int i=0; i < NCHAN*frame_count; i++)
	  Output_buffer[opp++] += residual[i] * sp->gain;
	pthread_mutex_unlock(&Buffer_mutex);      
	
	residual_frames -= frame_count;
	frames_needed -= frame_count;
	if(residual_frames != 0){
	  // Not completely exhaused; save for next iteration
	  memmove(residual,
		  &residual[NCHAN*frame_count],
		  NCHAN*residual_frames*sizeof(float));
	}
      }
      if(frames_needed <= 0) // Residual met the need
	break;
      
      // Need more, run the decoder
      assert(residual_frames == 0);

      // Extract block from queue, if any
      pthread_mutex_lock(&sp->qmutex);
      struct queue *qp = sp->queue;
      if(qp != NULL){
	sp->queue = qp->next;
	if(sp->eseq != qp->rtp.seq){
	  int drops = (signed short)(qp->rtp.seq - sp->eseq); // Sequence numbers are 16 bits
	  if(drops > 0 && drops < 1000) // Arbitrary sanity check
	    sp->drops += drops;
	}
	sp->eseq = qp->rtp.seq + 1;
      } else
	qp = NULL;
      pthread_mutex_unlock(&sp->qmutex);

      int size;
      if(qp != NULL){
	switch(qp->rtp.mpt){
	case 10: // Stereo PCM; 2 bytes = 1/2 frame
	  for(int i=0; i<qp->len/2; i++){
	    signed short s = (qp->data[2*i] << 8) + qp->data[2*i+1];
	    residual[i] = s * SCALE;
	  }
	  size = qp->len/4;
	  break;
	case 11: // Mono PCM, convert to "stereo"; 2 bytes = 1 frame
	  for(int i=0; i<qp->len/2; i++){
	    signed short s = (qp->data[2*i] << 8) + qp->data[2*i+1];
	    residual[2*i] = s * SCALE;
	    residual[2*i+1] = residual[2*i];
	  }
	  size = qp->len/2;
	  break;
	case 20: // Opus
	  size = opus_decode_float(sp->opus,qp->data,qp->len,residual,Output_size,0);
	  break;
	}
	free(qp->data);
	free(qp);
      } else {
	// Call decoder with erasure
	sp->underruns++;
	// Assume one lost packet of same size as before
	switch(sp->type){
	case 10:
	  size = 0;
	  break;
	case 11:
	  size = 0;
	  break;
	case 20:
	  size = opus_decode_float(sp->opus,NULL,0,residual,sp->opus_frame_size,0);
	  break;
	}
      }
      if(size <= 0) // Decoder error; give up
	break;
      residual_frames = size;
    }      
  }
}
