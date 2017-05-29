// $Id: audio.h,v 1.2 2016/10/13 23:27:44 karn Exp karn $
#ifndef _AUDIO_H
#define _AUDIO_H 1

#include <alsa/asoundlib.h>

struct audio {
  char *name;
  snd_pcm_t *handle;
  int samprate;
  int channels; // Number of channels in current buffer
  int underrun;
  int overflow;
  int echo;     // If set, echo to standard output
};

extern struct audio Audio;

int audio_change_parms(unsigned samplerate,int channels,int L);
int audio_out_done();
int put_stereo_audio(complex float *,int,float);
int put_mono_audio(float *,int,float);

#endif


