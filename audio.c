// $Id: audio.c,v 1.49 2017/10/01 23:43:56 karn Exp karn $
// Audio multicast routines for KA9Q SDR receiver
// Handles linear 16-bit PCM, mono and stereo, and the Opus lossy codec
// Copyright 2017 Phil Karn, KA9Q

#define _GNU_SOURCE 1
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <complex.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <complex.h>
#undef I
#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <opus/opus.h>
#include <portaudio.h>

#include "rtp.h"
#include "dsp.h"
#include "audio.h"
#include "multicast.h"

#define PCM_BUFSIZE 512        // 16-bit word count; must fit in Ethernet MTU

static short const scaleclip(float const x){
  if(x >= 1.0)
    return SHRT_MAX;
  else if(x <= -1.0)
    return SHRT_MIN;
  return (short)(SHRT_MAX * x);
}
  

// Portaudio callback - transfer data (if any) to provided buffer
// When buffer is empty, pad with silence
static int pa_callback(const void *inputBuffer, void *outputBuffer,
		       unsigned long framesPerBuffer,
		       const PaStreamCallbackTimeInfo* timeInfo,
		       PaStreamCallbackFlags statusFlags,
		       void *userData){
  complex float *op = outputBuffer;
  struct audio * const audio = userData;

  int samples_left = framesPerBuffer; // A stereo frame is two samples
  pthread_mutex_lock(&audio->buffer_mutex);
  while(samples_left > 0){
    // chunk is the smallest of the samples available, needed and the amount available before wrap
    int chunk = (audio->write_ptr - audio->read_ptr) & (AUD_BUFSIZE - 1);
    if(chunk > AUD_BUFSIZE - audio->read_ptr)
       chunk = AUD_BUFSIZE - audio->read_ptr;
    if(chunk > samples_left)
      chunk = samples_left;
    if(chunk == 0)
      break; // Nothing to send!
    memcpy(op,&audio->audiobuffer[audio->read_ptr],chunk * sizeof(*op));
    //    memset(&audio->buffer[audio->read],0,chunk * sizeof(*op)); // Zero buffer just read
    op += chunk;
    samples_left -= chunk;
    // Update the read pointer
    audio->read_ptr = (audio->read_ptr + chunk) & (AUD_BUFSIZE - 1); // Assumes Aud_bufsize is power of 2!!
  }
  pthread_mutex_unlock(&audio->buffer_mutex);
  memset(op,0,samples_left * sizeof(*op)); // Pad with silence
  return 0;
}

// Send 'size' stereo samples, each in a complex float
int send_stereo_audio(struct audio * const audio,complex float const * const buffer,int const size){
  // Write to circular buffer for Opus, PCM and/or portaudio
  pthread_mutex_lock(&audio->buffer_mutex);
  for(int i=0; i < size; i++){
    audio->audiobuffer[audio->write_ptr++] = buffer[i];
    if(audio->write_ptr >= AUD_BUFSIZE)
      audio->write_ptr -= AUD_BUFSIZE;
  }
  pthread_cond_broadcast(&audio->buffer_cond);
  pthread_mutex_unlock(&audio->buffer_mutex);

  if(audio->stream){
    // Write to file stream
    for(int i=0;i<size;i++){
      int16_t sample;

      sample = scaleclip(crealf(buffer[i]));
      fwrite(&sample,sizeof(sample),1,audio->stream);
      sample = scaleclip(cimagf(buffer[i]));
      fwrite(&sample,sizeof(sample),1,audio->stream);
    }
  }
  return 0;
}

// Send 'size' mono samples, each in a float
int send_mono_audio(struct audio * const audio,float const * const buffer,int const size){
  // Write to circular buffer for Opus, PCM and/or portaudio
  pthread_mutex_lock(&audio->buffer_mutex);
  for(int i=0; i < size; i++){
    audio->audiobuffer[audio->write_ptr++] = CMPLXF(buffer[i],buffer[i]);
    if(audio->write_ptr >= AUD_BUFSIZE)
      audio->write_ptr -= AUD_BUFSIZE;
  }
  pthread_cond_broadcast(&audio->buffer_cond);
  pthread_mutex_unlock(&audio->buffer_mutex);

  if(audio->stream){
    // Stream in stereo for consistency
    for(int i=0;i<size;i++){
      int16_t sample;
      sample = scaleclip(buffer[i]);
      fwrite(&sample,sizeof(sample),1,audio->stream);
      fwrite(&sample,sizeof(sample),1,audio->stream);
    }
  }
  return 0;
}


// Thread for compressing and multicasting audio with Opus codec
void *stereo_opus_audio(void *arg){
  pthread_setname("opus");
  assert(arg != NULL);
  struct audio *audio = arg;
  uint32_t timestamp = 0;
  uint16_t seq = 0;
  time_t tt = time(NULL);
  uint32_t const ssrc = tt & 0xffffffff;

  // Must correspond to 2.5, 5, 10, 20, 40, 60 ms
  // i.e., 120, 240, 480, 960, 1920, 2880 samples @ 48 kHz
  // opus 1.2 also supports 80, 100 and 120 ms
  if(audio->opus_blocktime != 2.5 && audio->opus_blocktime != 5
     && audio->opus_blocktime != 10 && audio->opus_blocktime != 20
     && audio->opus_blocktime != 40 && audio->opus_blocktime != 60
     && audio->opus_blocktime != 80 && audio->opus_blocktime != 100
     && audio->opus_blocktime != 120){
    fprintf(stderr,"opus block time must be 2.5/5/10/20/40/60/80/100/120 ms\n");
    fprintf(stderr,"80/100/120 supported only on opus 1.2 and later\n");
    return NULL;
  }
  int const opus_blocksize = round(audio->opus_blocktime * audio->samprate / 1000.);
  int error;
  audio->opus = opus_encoder_create(audio->samprate,2,OPUS_APPLICATION_AUDIO,&error);
  if(audio->opus == NULL){
    fprintf(stderr,"opus_encoder_create failed, error %d\n",error);
    return NULL;
  }
  opus_encoder_ctl(audio->opus,OPUS_SET_BITRATE(audio->opus_bitrate));
  opus_encoder_ctl(audio->opus,OPUS_SET_DTX(audio->opus_dtx));
  opus_encoder_ctl(audio->opus,OPUS_SET_LSB_DEPTH(16));

  struct rtp_header rtp;
  rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
  rtp.ssrc = htonl(ssrc);
  rtp.mpt = 20;         // arbitrary choice

  struct iovec iovec[2];
  unsigned char data[2048]; // Larger than Ethernet MTU
  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = data;
  
  struct msghdr message;
  message.msg_name = NULL; // Set by connect() call in setup_output()
  message.msg_namelen = 0;
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 2;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  float const decay = expf(-0.5 * (float)opus_blocksize / audio->samprate);

  int read_ptr = 0;
  while(1){
    complex float opusbuf[sizeof(complex float) * opus_blocksize];
    for(int i = 0; i < opus_blocksize;){
      pthread_mutex_lock(&audio->buffer_mutex);
      while(audio->write_ptr == read_ptr)
	pthread_cond_wait(&audio->buffer_cond,&audio->buffer_mutex); // Block until something available
      int avail = audio->write_ptr - read_ptr; // How much is available
      pthread_mutex_unlock(&audio->buffer_mutex);  
      if(avail < 0)
	avail += AUD_BUFSIZE; // write has wrapped around behind read
      
      for(; avail > 0 && i < opus_blocksize; i++,avail--){
	opusbuf[i] = audio->audiobuffer[read_ptr++];
	if(read_ptr >= AUD_BUFSIZE)
	  read_ptr -= AUD_BUFSIZE;
      }
    }
    // Encoder accepts stereo, which we represent as complex, but it wants an array of floats
    int const dlen = opus_encode_float(audio->opus,(float *)opusbuf,opus_blocksize,data,sizeof(data));
    if(dlen < 0){
      fprintf(stderr,"opus encode error %d\n",dlen);
      continue;
    }
    // Don't transmit if discontinuous mode is selected and frame is <= 2 bytes
    if(!audio->opus_dtx || dlen > 2){
      rtp.seq = htons(seq++);
      rtp.timestamp = htonl(timestamp);
      iovec[1].iov_len = dlen; // Length varies
      int r = sendmsg(audio->audio_mcast_fd,&message,0);
      if(r < 0){
	perror("opus: sendmsg");
	break;
      }
      audio->audio_packets++;
      audio->bitrate = decay * ((decay * audio->bitrate) + 8 * dlen);
    } else {
      audio->bitrate = decay * decay * audio->bitrate;
    }
    // always update timestamp so decoder will know how much was dropped
    timestamp += opus_blocksize;
  }
  opus_encoder_destroy(audio->opus);
  audio->opus = NULL;
  return NULL;
}
void *pcm_audio(void *arg){
  pthread_setname("pcm");
  assert(arg != NULL);
  struct audio *audio = arg;

  uint32_t timestamp = 0;
  uint16_t seq = 0;
  time_t tt = time(NULL);
  uint32_t const ssrc = tt & 0xffffffff;

  struct rtp_header rtp;
  rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
  rtp.ssrc = htonl(ssrc);


  int16_t PCM_buf[PCM_BUFSIZE];

  struct iovec iovec[2];      
  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = PCM_buf;
  
  struct msghdr message;      
  message.msg_name = NULL; // Set by connect() call in setup_output()
  message.msg_namelen = 0;
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 2;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;
  
  float const decay = expf(-0.5 * (PCM_BUFSIZE/2)/ audio->samprate);
  int read_ptr = 0;
  int stereo = 0;
  while(1){
    for(int i = 0; i < PCM_BUFSIZE/2;){
      pthread_mutex_lock(&audio->buffer_mutex);
      while(audio->write_ptr == read_ptr)
	pthread_cond_wait(&audio->buffer_cond,&audio->buffer_mutex); // Block until something available
      int avail = audio->write_ptr - read_ptr; // How much is available
      pthread_mutex_unlock(&audio->buffer_mutex);  
      if(avail < 0)
	avail += AUD_BUFSIZE; // write has wrapped around behind read
      
      for(; avail > 0 && i < PCM_BUFSIZE/2; i++,avail--){
	complex float samp = audio->audiobuffer[read_ptr++];
	if(read_ptr >= AUD_BUFSIZE)
	  read_ptr -= AUD_BUFSIZE;

	PCM_buf[2*i] = htons(scaleclip(crealf(samp)));
	PCM_buf[2*i+1] = htons(scaleclip(cimagf(samp)));
	if(PCM_buf[2*i] != PCM_buf[2*i+1])
	  stereo = 1;
      }
    }
    audio->audio_packets++;
    rtp.seq = htons(seq++);
    rtp.timestamp = htonl(timestamp);
    timestamp += PCM_BUFSIZE/2; // Increase by stereo sample count
    // If packet appears to be mono, send it as such
    if(!stereo){
      // Compress down to mono
      for(int i=0;i<PCM_BUFSIZE/2;i++)
	PCM_buf[i] = PCM_buf[2*i];
      rtp.mpt = 11;         // 16 bit linear, big endian, mono
      iovec[1].iov_len = sizeof(PCM_buf)/2;
      audio->bitrate = decay * ((decay * audio->bitrate) + 8 * sizeof(PCM_buf)/2);
    } else {
      rtp.mpt = 10;         // 16 bit linear, big endian, stereo
      iovec[1].iov_len = sizeof(PCM_buf); // byte count - fixed
      audio->bitrate = decay * ((decay * audio->bitrate) + 8 * sizeof(PCM_buf));
    }
    int r = sendmsg(audio->audio_mcast_fd,&message,0);
    if(r < 0){
      perror("pcm: sendmsg");
      break;
    }
  }
  return NULL;
}

void audio_cleanup(void *p){
  struct audio * const audio = p;
  if(audio == NULL)
    return;

  if(audio->Pa_Stream != NULL){
    Pa_AbortStream(&audio->Pa_Stream);
    Pa_CloseStream(&audio->Pa_Stream);
    audio->Pa_Stream = NULL;
    Pa_Terminate();
  }
  pthread_cancel(audio->pcm_thread);
  pthread_join(audio->pcm_thread,NULL);
  pthread_cancel(audio->opus_stereo_thread);
  pthread_join(audio->opus_stereo_thread,NULL);

  if(audio->audio_mcast_fd > 0){
    close(audio->audio_mcast_fd);
    audio->audio_mcast_fd = -1;
  }
  if(audio->stream != NULL){
    fclose(audio->stream);
    audio->stream = NULL;
  }
}


static int setup_portaudio(struct audio *const audio){
  if(audio == NULL)
    return -1;

  if(strcmp(audio->localdev,"none") == 0)
    return 0; // special name to disable portaudio

  PaError r = Pa_Initialize();
  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));
    return r;
  }
  int inDevNum;
  if(strlen(audio->localdev) == 0){
    // not specified; use default
    inDevNum = Pa_GetDefaultOutputDevice();
  } else {
    // Find requested audio device in the list
    int numDevices = Pa_GetDeviceCount();
    
    for(inDevNum=0; inDevNum < numDevices; inDevNum++){
      const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(inDevNum);
      if(strcmp(deviceInfo->name,audio->localdev) == 0)
	break;
    }
  }
  if(inDevNum == paNoDevice){
    fprintf(stderr,"Portaudio: no available devices\n");
    return -1;
  }
  PaStreamParameters outputParameters;
  memset(&outputParameters,0,sizeof(outputParameters));
  outputParameters.channelCount = 2;
  outputParameters.device = inDevNum;
  outputParameters.sampleFormat = paFloat32;
  
  r = Pa_OpenStream(&audio->Pa_Stream,NULL,&outputParameters,48000,paFramesPerBufferUnspecified,
		    paNoFlag,pa_callback,&Audio);
  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));      
    return r;
  }
  r = Pa_StartStream(audio->Pa_Stream);
  if(r != paNoError){
    fprintf(stderr,"Portaudio error: %s\n",Pa_GetErrorText(r));
    return r;
  }
  return 0;
}


// Set up pipes to encoding/sending tasks and start them up
int setup_audio(struct audio * const audio){
  assert(audio != NULL);
  pthread_mutex_init(&audio->buffer_mutex,NULL);
  pthread_cond_init(&audio->buffer_cond,NULL);

  setup_portaudio(audio);
  audio->bitrate = 0;

  if(audio->opus_bitrate != 0 || audio->rtp_pcm != 0){
    // Set up audio output stream(s)
    if(audio->audio_mcast_fd > 0)
      close(audio->audio_mcast_fd);
    audio->audio_mcast_fd = setup_mcast(audio->audio_mcast_address_text,1);
    if(audio->audio_mcast_fd == -1){
      fprintf(stderr,"Can't set up multicast audio output\n");
      return -1;
    }
    if(audio->opus_bitrate != 0)
      pthread_create(&audio->opus_stereo_thread,NULL,stereo_opus_audio,audio);
    if(audio->rtp_pcm != 0)
      pthread_create(&audio->pcm_thread,NULL,pcm_audio,audio);
  }    
  return 0;
}
