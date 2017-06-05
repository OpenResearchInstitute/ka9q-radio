// $Id: audio.h,v 1.4 2017/06/04 10:54:38 karn Exp karn $
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
  int stereo_input;
  int mono_input;
};

extern struct audio Audio;
extern int Audio_mono_sock;
extern int Audio_stereo_sock;
int audio_change_parms(unsigned samplerate,int channels,int L);
int audio_out_done();
void *audio_thread(void *);
pthread_t Audio_thread;

#endif


