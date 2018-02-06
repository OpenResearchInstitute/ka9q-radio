// $Id: funcube.c,v 1.25 2017/10/20 22:43:33 karn Exp karn $
// Read from AMSAT UK Funcube Pro and Pro+ dongles
// Multicast raw 16-bit I/Q samples
// Accept control commands from UDP socket
#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <stdio.h>
#include <alsa/asoundlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <locale.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/resource.h>

#include "fcd.h"
#include "sdr.h"
#include "radio.h"
#include "misc.h"
#include "multicast.h"

void *display(void *arg);
pthread_t Display_thread;


struct sdrstate {
  // Stuff for sending commands
  void *phd;                 // Opaque pointer to type hid_device

  struct status status;     // Frequency and gain settings, grouped for transmission in RTP packet

  // Analog gain settings
  int agc_holdoff;
  int agc_holdoff_count;

  float power;              // Running estimate of signal power - used only by display

  // ALSA parameters
  snd_pcm_t *sdr_handle;     // ALSA handle
  snd_pcm_hw_params_t *sdrparams; // ALSA parameters for A/D converter
  char sdr_name[50];         // ALSA name of associated audio device for A/D
  int overrun;               // A/D overrun count
};

struct sdrstate FCD;
pthread_t FCD_control_thread;
int ADC_samprate = 192000;
int Verbose;
int No_hold_open; // if set, close control between commands
int Dongle;       // Which of several funcube dongles to use
int Blocksize = 256;      // Could be up to 356 bytes with 20 bytes available for tunneling, but this is a power of 2
int Dongle = 0;
char *Locale;


void *fcd_command(void *arg);
int process_fc_command(char *,int);
double set_fc_LO(double);

int Rtp_sock; // Socket handle for sending real time stream *and* receiving commands
int Ctl_sock;
extern int Mcast_ttl;


int main(int argc,char *argv[]){
  // if we have root, up our priority and drop privileges
  int prio = getpriority(PRIO_PROCESS,0);
  prio = setpriority(PRIO_PROCESS,0,prio - 10);

  // Quickly drop root if we have it
  // The sooner we do this, the fewer options there are for abuse
  seteuid(getuid());


  struct rtp_header rtp;
  char *dest = "239.1.2.1"; // Default for testing

  Locale = getenv("LANG");
  if(Locale == NULL || strlen(Locale) == 0)
    Locale = "en_US.UTF-8";

  int c;
  while((c = getopt(argc,argv,"d:vp:l:b:oR:T:")) != EOF){
    switch(c){
    case 'R':
      dest = optarg;
      break;
    case 'o':
      No_hold_open++; // Close USB control port between commands so fcdpp can be used
      break;
    case 'd':
      Dongle = strtol(optarg,NULL,0);
      break;
    case 'v':
      Verbose++;
      break;
    case 'l':
      Locale = optarg;
      break;
    case 'b':
      Blocksize = strtol(optarg,NULL,0);
      break;
    case 'T':
      Mcast_ttl = strtol(optarg,NULL,0);
      break;
    }
  }
  if(Verbose)
    fprintf(stderr,"funcube dongle %d: blocksize %d\n",Dongle,Blocksize);

  setlocale(LC_ALL,Locale);
  
  // Set up RTP output socket
  Rtp_sock = setup_mcast(dest,1);
  if(Rtp_sock == -1){
    perror("Can't create multicast socket");
    exit(1);
  }
    
  // Set up control socket
  Ctl_sock = socket(AF_INET,SOCK_DGRAM,0);

  // bind control socket to next sequential port after our multicast source port
  struct sockaddr_in ctl_sockaddr;
  socklen_t siz = sizeof(ctl_sockaddr);
  if(getsockname(Rtp_sock,&ctl_sockaddr,&siz) == -1){
    perror("getsockname on ctl port");
    exit(1);
  }
  struct sockaddr_in locsock;
  locsock.sin_family = AF_INET;
  locsock.sin_port = htons(ntohs(ctl_sockaddr.sin_port)+1);
  locsock.sin_addr.s_addr = INADDR_ANY;
  bind(Ctl_sock,&locsock,sizeof(locsock));

  if(front_end_init(Dongle,ADC_samprate,Blocksize) == -1){
    fprintf(stderr,"front_end_init(%d,%d,%d) failed\n",Dongle,ADC_samprate,Blocksize);
    exit(1);
  }
  usleep(100000); // Let things settle
  pthread_create(&FCD_control_thread,NULL,fcd_command,&FCD);
  
  signal(SIGPIPE,SIG_IGN);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        

  if(Verbose > 1)
    pthread_create(&Display_thread,NULL,display,NULL);

  time_t tt;
  time(&tt);
  long ssrc;
  ssrc = tt & 0xffffffff; // low 32 bits of clock time

  rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
  rtp.mpt = 97;         // ordinarily dynamically allocated
  rtp.ssrc = htonl(ssrc);

  short sampbuf[2*Blocksize];
  struct iovec iovec[3];
  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = &FCD.status;
  iovec[1].iov_len = sizeof(FCD.status);
  iovec[2].iov_base = sampbuf;
  iovec[2].iov_len = sizeof(sampbuf);

  struct msghdr message;
  message.msg_name = NULL;
  message.msg_namelen = 0;
  message.msg_iov = &iovec[0];
  message.msg_iovlen = 3;
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  int timestamp = 0;
  int seq = 0;
  while(1){

    get_adc(sampbuf,Blocksize);
    if(Verbose > 1){
      // Only used by display, so don't calculate if the display isn't running
      // average energy (I+Q) in each sample, current block, **including DC offset**
      // At low levels, will disagree with demod's IF1 figure, which has the DC removed
      float sumsq = 0;
      for(int i=0;i<2*Blocksize;i++)
	sumsq += (float)sampbuf[i] * sampbuf[i];

      FCD.power = sumsq/Blocksize;
    }

    rtp.seq = htons(seq++);
    rtp.timestamp = htonl(timestamp);
    timestamp += Blocksize;

    if(sendmsg(Rtp_sock,&message,0) == -1){
      perror("sendmsg");
      // If we're sending to a unicast address without a listener, we'll get ECONNREFUSED
      // Sleep 1 sec to slow down the rate of these messages
      usleep(1000000);
    }
  }
  // Can't really get here
  close(Rtp_sock);
  exit(0);
}


int front_end_init(int dongle, int samprate,int L){
  int r;
  unsigned int exact_rate;
  snd_pcm_uframes_t buffer_size;

  FCD.status.samprate = samprate;
  FCD.agc_holdoff = 0.5 * FCD.status.samprate / L; // Block AGC changes for 1.0 sec after each change - samprate might not be set yet?
  FCD.agc_holdoff_count = 0;

  if(Verbose)
    fprintf(stderr,"Funcube dongle: ");
  if((FCD.phd = fcdOpen(FCD.sdr_name,sizeof(FCD.sdr_name),dongle)) == NULL){
    perror("fcdOpen()");
    return -1;
  }
  if((r = fcdGetMode(FCD.phd)) == FCD_MODE_APP){
    char caps_str[100];
    fcdGetCapsStr(FCD.phd,caps_str);
    if(Verbose)
      fprintf(stderr,"ALSA name '%s', caps '%s'\n",FCD.sdr_name,caps_str);
  } else if(r == FCD_MODE_NONE){
    fprintf(stderr," No FCD detected!\n");
    r = -1;
    goto done;
  } else if (r == FCD_MODE_BL){
    fprintf(stderr," is in bootloader mode\n");
    r = -1;
    goto done;
  }
  // Load current tuner state
  int intfreq;
  fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_FREQ_HZ,(unsigned char *)&intfreq,sizeof(intfreq));
  FCD.status.frequency = intfreq;

  fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_LNA_GAIN,&FCD.status.lna_gain,sizeof(FCD.status.lna_gain));
  fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_MIXER_GAIN,&FCD.status.mixer_gain,sizeof(FCD.status.mixer_gain));
  fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_IF_GAIN1,&FCD.status.if_gain,sizeof(FCD.status.if_gain));


  // Set up sample stream through ALSA subsystem
  if(Verbose)
    fprintf(stderr,"adc_setup(%s): ",FCD.sdr_name);
  if((r = snd_pcm_open(&FCD.sdr_handle,FCD.sdr_name,SND_PCM_STREAM_CAPTURE,0)) < 0){
    fprintf(stderr,"error opening PCM device: %s\n",snd_strerror(r));
    perror("");
    r = -1;
    goto done;
  }

  if(FCD.sdrparams == NULL)
    snd_pcm_hw_params_malloc(&FCD.sdrparams);
  if(snd_pcm_hw_params_any(FCD.sdr_handle,FCD.sdrparams) < 0){
    perror("can't configure this PCM device");
    r = -1;
    goto done;
  }
  if(snd_pcm_hw_params_set_access(FCD.sdr_handle,FCD.sdrparams,SND_PCM_ACCESS_RW_INTERLEAVED) < 0){
    perror("error setting access");
    r = -1;
    goto done;
  }
  if(snd_pcm_hw_params_set_format(FCD.sdr_handle,FCD.sdrparams,SND_PCM_FORMAT_S16_LE)<0){
    perror("error setting format");
    r = -1;
    goto done;
  }
  exact_rate = FCD.status.samprate;
  if(snd_pcm_hw_params_set_rate_near(FCD.sdr_handle,FCD.sdrparams,&exact_rate,0) < 0){
    perror("error setting rate");
    r = -1;
    goto done;
  }
  FCD.status.samprate = exact_rate;
  if(snd_pcm_hw_params_set_channels(FCD.sdr_handle,FCD.sdrparams,2) < 0){
    perror("error setting channels");
    r = -1;
    goto done;
  }
  // We will generally read L-sample blocks at a time
  snd_pcm_uframes_t LL = L;
  if(snd_pcm_hw_params_set_period_size_near(FCD.sdr_handle,FCD.sdrparams,&LL,0) < 0){
    perror("error setting periods");
    r = -1;
    goto done;
  }
  buffer_size = 1<<18;
  if(snd_pcm_hw_params_set_buffer_size_near(FCD.sdr_handle,FCD.sdrparams,&buffer_size) < 0){
    perror("error setting buffersize");
    r = -1;
    goto done;
  }
  if(snd_pcm_hw_params(FCD.sdr_handle,FCD.sdrparams) < 0){
    perror("error setting HW params");
    r = -1;
    goto done;
  }
  if(Verbose)
    fprintf(stderr,"A/D buffer %'d complex samples (%'.1f ms @ %'lu S/s)\n",
	    (int)buffer_size,
	    1000.*(float)buffer_size/FCD.status.samprate,
	    (unsigned long)FCD.status.samprate);

  snd_pcm_prepare(FCD.sdr_handle); // Start A/D conversion
  r = 0;

 done:; // Also the abort target: close handle before returning
  if(No_hold_open && FCD.phd != NULL){
    fcdClose(FCD.phd);
    FCD.phd = NULL;
  }
  return r;
}

// Read buffer of samples from front end
// L is number of stero samples, so buffer must have 2*L elements
int get_adc(short *buffer,const int L){
  int r;

  // Read block of I/Q samples from A/D converter
  do {
    snd_pcm_state_t state;
    state = snd_pcm_state(FCD.sdr_handle);
    if(state != SND_PCM_STATE_RUNNING && state != SND_PCM_STATE_PREPARED){
      FCD.overrun++;
      snd_pcm_prepare(FCD.sdr_handle);
    }
    if((r = snd_pcm_readi(FCD.sdr_handle,buffer,L)) < 0){
      fprintf(stderr,"funcube read error %s, reinit...\n",snd_strerror(r));
      front_end_init(Dongle,ADC_samprate,Blocksize);
      usleep(500000); // Just to keep from locking things up
    }
  } while(r != L);
  return 0;
}

// Process commands to change FCD state
// We listen on the same IP address and port we use as a multicasting source
void *fcd_command(void *arg){
  pthread_setname("funcube-cmd");

  while(1){
    fd_set fdset;
    socklen_t addrlen;
    int r;
    struct timeval timeout;
    struct status requested_status;
    
    // We're only reading one socket, but do it with a timeout so
    // we can periocally poll the Funcube Pro's status in case it
    // has been changed by another program, e.g., fcdpp
    FD_ZERO(&fdset);
    FD_SET(Ctl_sock,&fdset);
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    r = select(Ctl_sock+1,&fdset,NULL,NULL,&timeout);
    if(r == -1){
      perror("select");
      sleep(50000); // don't loop tightly
      continue;
    } else if(r == 0){
      // Timeout: poll the FCD for its current state, in case it
      // has changed independently, e.g., from fcdctl command
      if(FCD.phd == NULL && (FCD.phd = fcdOpen(FCD.sdr_name,sizeof(FCD.sdr_name),Dongle)) == NULL){
	perror("funcube: can't re-open control port");
	sleep(50000);
	continue;
      }

      fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_LNA_GAIN,&FCD.status.lna_gain,sizeof(FCD.status.lna_gain));
      fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_MIXER_GAIN,&FCD.status.mixer_gain,sizeof(FCD.status.mixer_gain));
      fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_IF_GAIN1,&FCD.status.if_gain,sizeof(FCD.status.if_gain));
      int intfreq;
      fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_FREQ_HZ,(unsigned char *)&intfreq,sizeof(intfreq));
      FCD.status.frequency = intfreq;
      goto done;
    } 
    // A command arrived, read it
    // Should probably log these
    struct sockaddr_in6 command_address;
    addrlen = sizeof(command_address);
    if((r = recvfrom(Ctl_sock,&requested_status,sizeof(requested_status),0,&command_address,&addrlen)) <= 0){
      if(r < 0)
	perror("recv");
      sleep(50000); // don't loop tightly
      continue;
    }
      
    if(r < sizeof(requested_status))
      continue; // Too short; ignore
    
    if(FCD.phd == NULL && (FCD.phd = fcdOpen(FCD.sdr_name,sizeof(FCD.sdr_name),Dongle)) == NULL){
      perror("funcube: can't re-open control port, aborting");
      abort();
    }      

    // See what has changed, and set it in the hardware
    if(requested_status.lna_gain != 0xff && FCD.status.lna_gain != requested_status.lna_gain){
      FCD.status.lna_gain = requested_status.lna_gain;
#if 0
      fprintf(stderr,"lna %s\n",FCD.status.lna_gain ? "ON" : "OFF");
#endif
      fcdAppSetParam(FCD.phd,FCD_CMD_APP_SET_LNA_GAIN,&FCD.status.lna_gain,sizeof(FCD.status.lna_gain));
    }
    if(requested_status.mixer_gain != 0xff && FCD.status.mixer_gain != requested_status.mixer_gain){
      FCD.status.mixer_gain = requested_status.mixer_gain;
#if 0
      fprintf(stderr,"mixer %s\n",FCD.status.mixer_gain ? "ON" : "OFF");
#endif
      fcdAppSetParam(FCD.phd,FCD_CMD_APP_SET_MIXER_GAIN,&FCD.status.mixer_gain,sizeof(FCD.status.mixer_gain));
    }
    if(requested_status.if_gain != 0xff && FCD.status.if_gain != requested_status.if_gain){
      FCD.status.if_gain = requested_status.if_gain;
#if 0
      fprintf(stderr,"IF gain %d db\n",FCD.status.if_gain);
#endif
      fcdAppSetParam(FCD.phd,FCD_CMD_APP_SET_IF_GAIN1,&FCD.status.if_gain,sizeof(FCD.status.if_gain));
    }
    if(requested_status.frequency > 0 && FCD.status.frequency != requested_status.frequency){
      int intfreq;
      
#if 0
      fprintf(stderr,"tuner frequency %lf\n",requested_status.frequency);
#endif
      intfreq = FCD.status.frequency = requested_status.frequency;
      fcdAppSetFreq(FCD.phd,intfreq);
    }
  done:;
    if(No_hold_open && FCD.phd != NULL){
      fcdClose(FCD.phd);
      FCD.phd = NULL;
    }
  }
}

// Status display thread
void *display(void *arg){
  pthread_setname("funcube-disp");
  float powerdB;

  fprintf(stderr,"Frequency      LNA  Mix   IF   A/D\n");
  fprintf(stderr,"Hz                        dB  dBFS\n");

  while(1){
    // This is +3dB for both channels carrying a maximum amplitude square wave
    powerdB = 10*log10f(FCD.power) - 90.309; // voltage ratio of 32768 -> +90.309 dB

    fprintf(stderr,"%'-15.0lf%3d%5d%5d%'6.1f\r",
	    FCD.status.frequency,
	    FCD.status.lna_gain,
	    FCD.status.mixer_gain,
	    FCD.status.if_gain,
	    powerdB);
    usleep(100000); // 10 Hz
  }
  return NULL;
}



// If we don't stop the A/D, it'll take several seconds to overflow and stop by itself,
// and during that time we can't restart
void closedown(int a){
  if(Verbose)
    fprintf(stderr,"funcube: caught signal %d: %s\n",a,strsignal(a));
  snd_pcm_drop(FCD.sdr_handle);
  exit(1);
}
