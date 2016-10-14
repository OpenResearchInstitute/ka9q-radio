// $Id: audio.c,v 1.2 2016/10/13 23:26:43 karn Exp karn $
// Send PCM audio to Linux ALSA driver and/or as .wav stream on stdout
#include <assert.h>
#include <limits.h>
#include <string.h>
#include <complex.h>
#include <stdio.h>
#include <alsa/asoundlib.h>


#include "dsp.h"
#include "audio.h"

// I tried libsndfile, but it refuses to write .wav to a pipe
// Strictly speaking, it's entitled to do that. But everybody *else* does it,
// including sox and opusenc. Opusenc accepts only raw and wav,
// and I don't want to have to manually specify the parameters for a raw PCM stream...
// Why are digital audio standards so screwed up?

struct wavhdr {
  char riff[4];
  int chunksize1;
  char wave[4];
  char fmt[4];
  int chunksize2;
  short format;
  short channels;
  int samprate;
  int datarate;
  short blocksize;
  short bits;
  char data[4];
  int datalen;
};

struct audio Audio;

int audio_out_done(){
  snd_pcm_drop(Audio.handle);
#if 0
  snd_pcm_close(Audio.handle);
  Audio.handle = NULL;
#endif
  return 0;
}

// Set up or change ALSA for demodulated sound output
int audio_change_parms(unsigned samprate,int channels,int L){
  snd_pcm_uframes_t buffer_size;
  unsigned actual_rate;
  snd_pcm_hw_params_t *hw_params;
  snd_pcm_sw_params_t *sw_params;

#if 0
  // This is a nice informative message, but it gets called every time we change modes
  fprintf(stderr,"audio_change_parms(%s,rate=%u,blksize=%d,chans=%d)\n",Audio.name,samprate,L,channels);
#endif
  if(Audio.handle != NULL){
    // See if it's already as we like it
    if(Audio.channels == channels && Audio.samprate == samprate)
      return 0;
    snd_pcm_drop(Audio.handle);
    snd_pcm_close(Audio.handle);
    Audio.handle = NULL;
  }
  if(Audio.echo){
    struct wavhdr wavhdr;

    strcpy(wavhdr.riff,"RIFF");
    wavhdr.chunksize1 = 0x7fffffff; // infinity (nonstandard?)
    strcpy(wavhdr.wave,"WAVE");
    strcpy(wavhdr.fmt,"fmt ");
    wavhdr.chunksize2 = 16;
    wavhdr.format = 1;
    wavhdr.channels = channels;
    wavhdr.samprate = samprate;
    wavhdr.datarate = sizeof(short) * channels * samprate;
    wavhdr.blocksize = sizeof(short) * channels;
    wavhdr.bits = 16;
    strcpy(wavhdr.data,"data");
    wavhdr.datalen = 0x7fffffff;  // infinity (nonstandard?)
    write(1,&wavhdr,sizeof(wavhdr));
  }
  if(snd_pcm_open(&Audio.handle,Audio.name,SND_PCM_STREAM_PLAYBACK,0) < 0){
    fprintf(stderr,"Error opening D/A %s\n",Audio.name);
    return -1;
  }

  snd_pcm_hw_params_alloca(&hw_params);
  if(snd_pcm_hw_params_any(Audio.handle,hw_params) < 0){
    fprintf(stderr,"Can not configure D/A %s\n",Audio.name);
    return -1;
  }
  if(snd_pcm_hw_params_set_access(Audio.handle,hw_params,SND_PCM_ACCESS_RW_INTERLEAVED) < 0){
    fprintf(stderr,"Error setting D/A access on %s\n",Audio.name);
    return -1;
  }
  if(snd_pcm_hw_params_set_format(Audio.handle,hw_params,SND_PCM_FORMAT_S16_LE)<0){
    fprintf(stderr,"Error setting D/A format on %s\n",Audio.name);
    return -1;
  }
  actual_rate = samprate;
  if(snd_pcm_hw_params_set_rate_near(Audio.handle,hw_params,&actual_rate,0)<0){
    fprintf(stderr,"Error setting D/A sample rate on %s\n",Audio.name);
    return -1;
  }
  if(actual_rate != samprate)
    fprintf(stderr,"Warning: actual D/A sample rate is %u\n",actual_rate);

  Audio.samprate = actual_rate;
  if(snd_pcm_hw_params_set_channels(Audio.handle,hw_params,channels)<0){ // stereo
    fprintf(stderr,"Error setting D/A channels on %s\n",Audio.name);
    return -1;
  }
  // We will generally write L-sample blocks at a time
  snd_pcm_uframes_t LL = L;
  if(snd_pcm_hw_params_set_period_size_near(Audio.handle,hw_params,&LL,0)<0){
    fprintf(stderr,"Error setting D/A periods on %s\n",Audio.name);
    return -1;
  }
  buffer_size = 2*LL;
  if(snd_pcm_hw_params_set_buffer_size_near(Audio.handle,hw_params,&buffer_size)<0){
    fprintf(stderr,"Error setting D/A buffersize on %s\n",Audio.name);
    return -1;
  }
  if(snd_pcm_hw_params(Audio.handle,hw_params)<0){
    fprintf(stderr,"Error setting D/A HW params on %s\n",Audio.name);
    return -1;
  }
  snd_pcm_sw_params_alloca(&sw_params);
  if(snd_pcm_sw_params_current(Audio.handle,sw_params) < 0){
    fprintf(stderr,"Can not configure driver for %s\n",Audio.name);
    return -1;
  }
  if(snd_pcm_sw_params_set_start_threshold(Audio.handle,sw_params,1) < 0){
    fprintf(stderr,"Can not configure start threshold for %s\n",Audio.name);
    return -1;
  }
  if(snd_pcm_sw_params(Audio.handle,sw_params) < 0){
    fprintf(stderr,"Can not set sw params %s\n",Audio.name);
  }
  return 0;
}

int put_stereo_audio(complex float *buffer,int L,float gain){
  signed short outsamps[2*L];
  int cnt = L;
  int start,n;

  start = 0;
  for(n=0;n<L;n++){
    outsamps[2*n] = CLIP(SHRT_MAX*gain*creal(buffer[n]));
    outsamps[2*n+1] = CLIP(SHRT_MAX*gain*cimag(buffer[n]));
  }
  if(Audio.echo)
    write(1,outsamps,2*L*sizeof(short));

  while(cnt > 0){
    int chunk,r;
    snd_pcm_state_t state;

    state = snd_pcm_state(Audio.handle);
    // Underruns can deliberately happen when the demodulator thread simply
    // stops sending data, e.g., when a FM squelch is closed
    if(state != SND_PCM_STATE_RUNNING && state != SND_PCM_STATE_PREPARED){
      Audio.underrun++;
      snd_pcm_prepare(Audio.handle);
    }
    chunk = snd_pcm_avail(Audio.handle); // Don't send more than it can take!
    if(chunk == 0){
      // Wait for some room to open up 
      usleep((int)(512e6 / Audio.samprate));
      continue;
    }
    chunk = min(chunk,cnt);
    if((r = snd_pcm_writei(Audio.handle,&outsamps[start],chunk)) != chunk){
      fprintf(stderr,"audio write fail %s %d %s\n",snd_strerror(r),r,strerror(-r));
      snd_pcm_prepare(Audio.handle);
    }
    cnt -= chunk;
    start += 2*chunk;
  }
  return 0;
}
int put_mono_audio(float *buffer,int L,float gain){
  signed short outsamps[2*L];
  int cnt = L;
  int start,n;

  start = 0;
  for(n=0;n<L;n++){
    outsamps[2*n] = CLIP(SHRT_MAX*gain*buffer[n]);
    outsamps[2*n+1] = CLIP(SHRT_MAX*gain*buffer[n]);    
  }
  if(Audio.echo)
    write(1,outsamps,2*L*sizeof(short));

  while(cnt > 0){
    int chunk,r;
    snd_pcm_state_t state;
    
    state = snd_pcm_state(Audio.handle);
    // Underruns can deliberately happen when the demodulator thread simply
    // stops sending data, e.g., when a FM squelch is closed
    if(state != SND_PCM_STATE_RUNNING && state != SND_PCM_STATE_PREPARED){
      Audio.underrun++;
      snd_pcm_prepare(Audio.handle);
    }
    chunk = snd_pcm_avail(Audio.handle); // Don't send more than it can take!
    if(chunk == 0){
      // Wait for some room to open up 
      usleep((int)(512e6 / Audio.samprate));
      continue;
    }
    chunk = min(chunk,cnt);
    if((r = snd_pcm_writei(Audio.handle,&outsamps[start],chunk)) != chunk){
      fprintf(stderr,"audio write fail %s %d %s\n",snd_strerror(r),r,strerror(-r));
      snd_pcm_prepare(Audio.handle);
    }
    cnt -= chunk;
    start += chunk;
  }
  return 0;
}



