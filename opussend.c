// $Id$
// Multicast local audio with Opus
// Copyright Feb 2018 Phil Karn, KA9Q
#define _GNU_SOURCE 1
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <locale.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <limits.h>
#include <string.h>
#include <opus/opus.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/resource.h>
#include <netdb.h>
#include <portaudio.h>


#include "misc.h"
#include "multicast.h"

// Global config variables
char *Input_device_text = "";
char *Mcast_output_address_text = "audio-opus-mcast.local";     // Multicast address we're sending to

int const Bufsize = 8192;     // Maximum samples/words per RTP packet - must be bigger than Ethernet MTU
int const Samprate = 48000;   // Too hard to handle other sample rates right now
                              // Opus will notice the actual audio bandwidth, so there's no real cost to this
int Verbose;                  // Verbosity flag (currently unused)

// Opus codec params (defaults)
float Opus_blocktime = 20;    // 20 ms, a reasonable default
int Opus_bitrate = 32;        // Opus stream audio bandwidth; default 32 kb/s
int const Channels = 2;       // Stereo - no penalty if the audio is actually mono, Opus will figure it out
int Discontinuous = 0;        // Off by default
// End of config stuff

OpusEncoder *Opus;
int Output_fd = -1;
float Audiodata[Bufsize];
pthread_mutex_t Buffer_mutex;
pthread_cond_t Buffer_cond;
int Samples_available;

static int pa_callback(const void *inputBuffer, void *outputBuffer,
		       unsigned long framesPerBuffer,
		       const PaStreamCallbackTimeInfo* timeInfo,
		       PaStreamCallbackFlags statusFlags,
		       void *userData);


void closedown(int s){
  Pa_Terminate();
  if(Opus != NULL)
    opus_encoder_destroy(Opus);
  
  if(Output_fd != -1)
    close(Output_fd);
  exit(0);
}


int main(int argc,char * const argv[]){
  // Try to improve our priority
  int prio = getpriority(PRIO_PROCESS,0);
  prio = setpriority(PRIO_PROCESS,0,prio - 15);

  // Drop root if we have it
  seteuid(getuid());

  setlocale(LC_ALL,getenv("LANG"));

  int c;
  while((c = getopt(argc,argv,"I:vR:B:o:xT:")) != EOF){
    switch(c){
    case 'T':
      Mcast_ttl = strtol(optarg,NULL,0);
      break;
    case 'v':
      Verbose++;
      break;
    case 'I':
      Input_device_text = optarg;
      break;
    case 'R':
      Mcast_output_address_text = optarg;
      break;
    case 'B':
      Opus_blocktime = strtod(optarg,NULL);
      break;
    case 'o':
      Opus_bitrate = strtol(optarg,NULL,0);
      break;
    case 'x':
      Discontinuous = 1;
      break;
    default:
      fprintf(stderr,"Usage: %s [-x] [-v] [-o bitrate] [-B blocktime] [-I input_mcast_address] [-R output_mcast_address][-T mcast_ttl]\n",argv[0]);
      fprintf(stderr,"Defaults: %s -o %d -B %.1f -I %s -R %s -T %d\n",argv[0],Opus_bitrate,Opus_blocktime,Input_device_text,Mcast_output_address_text,Mcast_ttl);
      exit(1);
    }
  }
  // Set up Opus encoder
  if(Opus_blocktime != 2.5 && Opus_blocktime != 5
     && Opus_blocktime != 10 && Opus_blocktime != 20
     && Opus_blocktime != 40 && Opus_blocktime != 60
     && Opus_blocktime != 80 && Opus_blocktime != 100
     && Opus_blocktime != 120){
    fprintf(stderr,"opus block time must be 2.5/5/10/20/40/60/80/100/120 ms\n");
    fprintf(stderr,"80/100/120 supported only on opus 1.2 and later\n");
    exit(1);
  }
  int Opus_frame_size = round(Opus_blocktime * Samprate / 1000.);
  if(Opus_bitrate < 500)
    Opus_bitrate *= 1000; // Assume it was given in kb/s

  int error = 0;
  Opus = opus_encoder_create(Samprate,Channels,OPUS_APPLICATION_AUDIO,&error);
  opus_encoder_ctl(Opus,OPUS_SET_DTX(Discontinuous));
  opus_encoder_ctl(Opus,OPUS_SET_BITRATE(Opus_bitrate));
  opus_encoder_ctl(Opus,OPUS_FRAMESIZE_ARG,Opus_blocktime);

  // Set up multicast transmit socket
  Output_fd = setup_mcast(Mcast_output_address_text,1);
  if(Output_fd == -1){
    fprintf(stderr,"Can't set up output on %s: %s\n",Mcast_output_address_text,strerror(errno));
    exit(1);
  }
  // Set up to transmit Opus RTP/UDP/IP
  struct iovec iovec_out[2];
  struct rtp_header rtp_out;
  unsigned char data_out[Bufsize];
  struct msghdr message_out;

  iovec_out[0].iov_base = &rtp_out;
  iovec_out[0].iov_len = sizeof(rtp_out);
  iovec_out[1].iov_base = data_out;
  // iovec_out[1].iov_len varies

  unsigned long otimestamp = 0;
  unsigned short oseq = 0;
  unsigned long ssrc = time(0);

  message_out.msg_name = NULL; // connected-mode socket already has destination
  message_out.msg_namelen = 0;
  message_out.msg_iov = &iovec_out[0];
  message_out.msg_iovlen = 2;
  message_out.msg_control = NULL;
  message_out.msg_controllen = 0;
  message_out.msg_flags = 0;

  // Set up audio input
  PaError r = Pa_Initialize();
  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));
    close(Output_fd);
    return r;
  }
  PaStreamParameters inputParameters;
  memset(&inputParameters,0,sizeof(inputParameters));
  inputParameters.channelCount = Channels;
  int inDevNum = Pa_GetDefaultInputDevice();
  inputParameters.device = inDevNum;
  inputParameters.sampleFormat = paFloat32;
  inputParameters.suggestedLatency = .001 * Opus_blocktime;
  
  PaStream *Pa_Stream;          // Portaudio stream handle
  r = Pa_OpenStream(&Pa_Stream,&inputParameters,NULL,Samprate,
		    Opus_frame_size,
		     0,pa_callback,NULL);

  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));      
    close(Output_fd);
    exit(1);
  }
  r = Pa_StartStream(Pa_Stream);
  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));
    close(Output_fd);
    exit(1);
  }
  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);
  signal(SIGPIPE,SIG_IGN);

  pthread_mutex_init(&Buffer_mutex,NULL);
  pthread_cond_init(&Buffer_cond,NULL);
  
  int sampsize = sizeof(float)*Channels;
  float fbuffer[Channels * Opus_frame_size];

  while(1){
    // Wait for audio input
    pthread_mutex_lock(&Buffer_mutex);
    while(Samples_available < Opus_frame_size)
      pthread_cond_wait(&Buffer_cond,&Buffer_mutex);
    memcpy(fbuffer,Audiodata,sizeof(fbuffer));
    Samples_available -= Opus_frame_size;
    memmove(Audiodata,Audiodata+sampsize*Opus_frame_size,sampsize*Samples_available);
      
    pthread_mutex_unlock(&Buffer_mutex);    
    
    int size = opus_encode_float(Opus,fbuffer,Opus_frame_size,data_out,sizeof(data_out));
    if(!Discontinuous || size > 2){
      iovec_out[1].iov_len = size;
      rtp_out.seq = htons(oseq++);
      rtp_out.mpt = 20; // Opus
      rtp_out.ssrc = htonl(ssrc);
      rtp_out.timestamp = htonl(otimestamp);
      size = sendmsg(Output_fd,&message_out,0);
    }
    otimestamp += Opus_frame_size;
  }
  opus_encoder_destroy(Opus);
  close(Output_fd);
  exit(0);
}

// Portaudio callback - encode and transmit audio
static int pa_callback(const void *inputBuffer, void *outputBuffer,
		       unsigned long framesPerBuffer,
		       const PaStreamCallbackTimeInfo* timeInfo,
		       PaStreamCallbackFlags statusFlags,
		       void *userData){

  pthread_mutex_lock(&Buffer_mutex);
  memcpy(Audiodata+Samples_available*Channels*sizeof(float),
	 inputBuffer,framesPerBuffer*Channels*sizeof(float));
  Samples_available += framesPerBuffer;
  pthread_cond_broadcast(&Buffer_cond);
  pthread_mutex_unlock(&Buffer_mutex);
  
  return paContinue;
}
