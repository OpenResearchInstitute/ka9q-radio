// $Id: funcube.c,v 1.7 2016/10/24 13:26:19 karn Exp karn $
// Read from AMSAT UK Funcube Pro and Pro+ dongles
// Correct for DC offset, I/Q gain and phase imbalance
// Emit complex float sample stream on stdout
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

#include "fcd.h"
#include "sdr.h"
#include "command.h"
#include "dsp.h"
#include "rtp.h"

#define MAXPKT 1500

void *display(void *arg);
pthread_t Display_thread;


struct sdrstate {
  // Stuff for sending commands
  void *phd;                 // Opaque pointer to type hid_device
  pthread_mutex_t mutex; // Allow only one command to the device at a time
  pthread_cond_t cond;
  int pending;               // set to ask task to update dongle

  struct status status;     // Frequency and gain settings
  // Analog gain settings
  int agc_holdoff;
  int agc_holdoff_count;

  float power;              // Running estimate of signal power

  // ALSA parameters
  snd_pcm_t *sdr_handle;     // ALSA handle
  snd_pcm_hw_params_t *sdrparams; // ALSA parameters for A/D converter
  char sdr_name[50];         // ALSA name of associated audio device for A/D
  int samprate;              // nominal A/D sample rate, usually 192 kHz for funcube dongle pro+
  int overrun;               // A/D overrun count
};

struct sdrstate FCD;
pthread_t FCD_control_thread;
const int ADC_samprate = 192000;
int Verbose;
int No_hold_open; // if set, close control between commands
int Dongle;

void *fcd_command(void *arg);
int process_fc_command(char *,int);
double set_fc_LO(double);

int main(int argc,char *argv[]){
  struct sockaddr_in6 address6;
  struct sockaddr_in address4;  
  char *locale;
  int c,ctl;
  int ctl_port = 4160;
  int blocksize = 350;
  double f = 147435000;
  int Dongle = 0;
  struct rtp_header rtp;
  int rtp_sock;
  char *dest = NULL;
  int dest_port = -1;
  int seq = 0;
  int timestamp = 0;
  long ssrc;
 
  locale = getenv("LANG");

  while((c = getopt(argc,argv,"d:vf:p:l:b:oR:P:")) != EOF){
    switch(c){
    case 'R':
      dest = optarg;
      break;
    case 'P':
      dest_port = atoi(optarg);
      break;
    case 'o':
      No_hold_open++; // Close USB control port between commands so fcdpp can be used
      break;
    case 'd':
      Dongle = atoi(optarg);
      break;
    case 'v':
      Verbose++;
      break;
    case 'l':
      locale = optarg;
      break;
    case 'b':
      blocksize = atoi(optarg);
      break;
    case 'f':
      f = atof(optarg);
      break;
    case 'p':
      ctl_port = atoi(optarg);
      break;
    }
  }
  if(Verbose){
    fprintf(stderr,"funcube dongle %d: blocksize %d, UDP control port %d\n",
	    Dongle,blocksize,ctl_port);
  }
  setlocale(LC_ALL,locale);
  signal(SIGPIPE,SIG_IGN);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  
  // Set up RTP output socket
  if(inet_pton(AF_INET,dest,&address4.sin_addr) == 1){
    // Destination is IPv4
    address4.sin_family = AF_INET;
    address4.sin_port = htons(dest_port);
    if((rtp_sock = socket(PF_INET,SOCK_DGRAM,0)) == -1){
      fprintf(stderr,"funcube: can't create IPv4 output socket\n");
      exit(1);
    }
    if(connect(rtp_sock,&address4,sizeof(address4)) != 0){
      perror("connect to IPv4 output address failed");
      exit(1);
    }
    if(IN_MULTICAST(&address4)){
      // Destination is multicast; join it
      struct group_req group_req;
      group_req.gr_interface = 0;
      memcpy(&group_req.gr_group,&address4,sizeof(address4));
      if(setsockopt(rtp_sock,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0){
	perror("setsockopt ipv4 multicast join group failed");
      }
    }
  } else if(inet_pton(AF_INET6,dest,&address6.sin6_addr) == 1){
    // Destination is IPv6
    address6.sin6_family = AF_INET6;
    address6.sin6_flowinfo = 0;  
    address6.sin6_port = htons(dest_port);
    address6.sin6_scope_id = 0;
    if((rtp_sock = socket(PF_INET6,SOCK_DGRAM,0)) == -1){
      fprintf(stderr,"funcube: can't create IPv6 output socket\n");
      exit(1);
    }
    if(connect(rtp_sock,&address6,sizeof(address6)) != 0){
      perror("connect to IPv6 address failed");
      exit(1);
    }
    if(IN6_IS_ADDR_MULTICAST(&address6)){
      // Destination is multicast; join it
      struct group_req group_req;
      group_req.gr_interface = 0;
      memcpy(&group_req.gr_group,&address6,sizeof(address6));
      if(setsockopt(rtp_sock,IPPROTO_IPV6,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0){
	perror("setsockopt ipv6 multicast join group failed");
      }
    }
  } else {
    fprintf(stderr,"Can't parse destination %s\n",dest);
    exit(1);
  }
  // Apparently works for both IPv4 and IPv6
  u_char loop = 1;
  if(setsockopt(rtp_sock,IPPROTO_IP,IP_MULTICAST_LOOP,&loop,sizeof(loop)) != 0){
    perror("setsockopt multicast loop failed");
  }
  u_char ttl = 1;
  if(setsockopt(rtp_sock,IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof(ttl)) != 0){
    perror("setsockopt multicast ttl failed");
  }
      
  // Set up control input socket
  // Will accept either IPv4 or IPv6
  if((ctl = socket(PF_INET6,SOCK_DGRAM, 0)) == -1){
    fprintf(stderr,"funcube: can't open control socket\n");
    exit(1);
  }
  address6.sin6_family = AF_INET6;
  address6.sin6_port = htons(ctl_port);
  address6.sin6_flowinfo = 0;
  address6.sin6_addr = in6addr_any;
  address6.sin6_scope_id = 0;
  if(bind(ctl,&address6,sizeof(address6)) != 0){
    fprintf(stderr,"funcube: control port bind failed\n");
    exit(1);
  }
  fcntl(ctl,F_SETFL,O_NONBLOCK);


  front_end_init(Dongle,ADC_samprate,blocksize);
  usleep(100000);
  set_fc_LO(f); // Must be done after init

  pthread_mutex_lock(&FCD.mutex);
  mirics_gain(FCD.status.frequency,
	      20,
	      &FCD.status.if_gain,
	      &FCD.status.lna_gain,
	      &FCD.status.mixer_gain); // start low
  FCD.pending = 1;
  pthread_cond_broadcast(&FCD.cond);
  pthread_mutex_unlock(&FCD.mutex);    
  
  if(Verbose > 1)
    pthread_create(&Display_thread,NULL,display,NULL);

  time_t tt;
  time(&tt);
  ssrc = tt & 0xffffffff; // low 32 bits of clock time

  rtp.vpxcc = (RTP_VERS << 6); // Version 2, padding = 0, extension = 0, csrc count = 0
  rtp.mpt = 10;         // L16, stereo, but implies 44100 Hz and big-endian byte order?
  rtp.ssrc = htonl(ssrc);

  short sampbuf[2*blocksize];
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

  while(1){

    socklen_t addrlen;
    char pktbuf[MAXPKT];
    int rdlen,i;
    
    while(addrlen = sizeof(address6), (rdlen = recvfrom(ctl,&pktbuf,sizeof(pktbuf),0,&address6,&addrlen)) > 0){
      // Commands have priority - is this right?
      process_fc_command(pktbuf,rdlen);
    }
    get_adc(sampbuf,blocksize);
    if(Verbose > 1){
      // Only used by display, so don't calculate if the display isn't running
      // average energy (I+Q) in each sample, current block, **including DC offset**
      // At low levels, will disagree with demod's IF1 figure, which has the DC removed
      float sumsq = 0;
      for(i=0;i<2*blocksize;i++){
	sumsq += (float)sampbuf[i] * sampbuf[i];
      }
      FCD.power = sumsq/blocksize;
    }

    rtp.seq = htons(seq++);
    rtp.timestamp = htonl(timestamp);
    timestamp += blocksize;


    sendmsg(rtp_sock,&message,0);
  }
  close(rtp_sock);
  close(ctl);
  exit(0);
}


int front_end_init(int dongle, int samprate,int L){
  int r;
  unsigned int exact_rate;
  snd_pcm_uframes_t buffer_size;

  FCD.samprate = samprate;
  FCD.agc_holdoff = 0.5 * FCD.samprate / L; // Block AGC changes for 1.0 sec after each change - samprate might not be set yet?
  FCD.agc_holdoff_count = 0;

  pthread_mutex_init(&FCD.mutex,NULL);
  if(Verbose)
    fprintf(stderr,"Funcube dongle: ");
  if((FCD.phd = fcdOpen(FCD.sdr_name,sizeof(FCD.sdr_name),dongle)) == NULL){
    fprintf(stderr,"fcdOpen() failed\n");
    return -1;
  }
  if((r = fcdGetMode(FCD.phd)) == FCD_MODE_APP){
    char caps_str[100];
    fcdGetCapsStr(FCD.phd,caps_str);
    if(Verbose)
      fprintf(stderr,"ALSA name '%s', caps '%s'\n",FCD.sdr_name,caps_str);
  } else if(r == FCD_MODE_NONE){
    fprintf(stderr," No FCD detected!\n");
    return -1;
  } else if (r == FCD_MODE_BL){
    fprintf(stderr," is in bootloader mode\n");
    return -1;
  }

  // Now set up sample stream through ALSA subsystem
  if(Verbose)
    fprintf(stderr,"adc_setup(%s): ",FCD.sdr_name);
  if((r = snd_pcm_open(&FCD.sdr_handle,FCD.sdr_name,SND_PCM_STREAM_CAPTURE,0)) < 0){
    fprintf(stderr,"error opening PCM device: %s\n",snd_strerror(r));
    return -1;
  }

  if(FCD.sdrparams == NULL)
    snd_pcm_hw_params_malloc(&FCD.sdrparams);
  if(snd_pcm_hw_params_any(FCD.sdr_handle,FCD.sdrparams) < 0){
    fprintf(stderr,"can't configure this PCM device\n");
    return -1;
  }
  if(snd_pcm_hw_params_set_access(FCD.sdr_handle,FCD.sdrparams,SND_PCM_ACCESS_RW_INTERLEAVED) < 0){
    fprintf(stderr,"error setting access\n");
    return -1;
  }
  if(snd_pcm_hw_params_set_format(FCD.sdr_handle,FCD.sdrparams,SND_PCM_FORMAT_S16_LE)<0){
    fprintf(stderr,"error setting format\n");
    return -1;
  }
  exact_rate = FCD.samprate;
  if(snd_pcm_hw_params_set_rate_near(FCD.sdr_handle,FCD.sdrparams,&exact_rate,0)<0){
    fprintf(stderr,"error setting rate\n");
    return -1;
  }
  FCD.samprate = exact_rate;
  if(snd_pcm_hw_params_set_channels(FCD.sdr_handle,FCD.sdrparams,2)<0){
    fprintf(stderr,"error setting channels\n");
    return -1;
  }
  // We will generally read L-sample blocks at a time
  snd_pcm_uframes_t LL = L;
  if(snd_pcm_hw_params_set_period_size_near(FCD.sdr_handle,FCD.sdrparams,&LL,0)<0){
    fprintf(stderr,"error setting periods\n");
    return -1;
  }
  buffer_size = 1<<18;
  if(snd_pcm_hw_params_set_buffer_size_near(FCD.sdr_handle,FCD.sdrparams,&buffer_size)<0){
    fprintf(stderr,"error setting buffersize\n");
    perror("");
    return -1;
  }
  if(snd_pcm_hw_params(FCD.sdr_handle,FCD.sdrparams)<0){
    fprintf(stderr,"error setting HW params\n");
    return -1;
  }
  if(Verbose)
    fprintf(stderr,"A/D buffer %'d complex samples (%'.1f ms @ %'u S/s)\n",
	  (int)buffer_size,
	  1000.*(float)buffer_size/FCD.samprate,
	  FCD.samprate);

  //  pthread_mutex_init(&FCD.buffer_mutex,NULL);
  snd_pcm_prepare(FCD.sdr_handle); // Start A/D conversion
  pthread_create(&FCD_control_thread,NULL,fcd_command,&FCD);

  return 0;
}

// Read buffer of samples from front end
// L is number of stero samples, so buffer must have 2*L elements
int get_adc(short *buffer,int L){
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
      fprintf(stderr,"funcube read error %s\n",snd_strerror(r));
      usleep(500000); // Just to keep from locking things up
    }
  } while(r != L);
  return 0;
}

int process_fc_command(char *cmdbuf,int len){
  
  if(len >= sizeof(struct command)){
    struct command command;

    memcpy(&command,cmdbuf,sizeof(command));
    switch(command.cmd){
    case SETSTATE:
      set_fc_LO(command.first_LO); // Only command we process
      break;
    default:
      break; // Ignore
    } // switch
  }
  return 0;
}


double set_fc_LO(double f){
  pthread_mutex_lock(&FCD.mutex);
  FCD.pending = 1;
  FCD.status.frequency = round(f);
  pthread_cond_broadcast(&FCD.cond);
  pthread_mutex_unlock(&FCD.mutex);    

  return FCD.status.frequency;
}

void *fcd_command(void *arg){
  unsigned char lna_gain,mixer_gain,if_gain;
  int intfreq;
  double frequency;

  // Load current tuner state
  pthread_mutex_lock(&FCD.mutex);
  fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_FREQ_HZ,(unsigned char *)&intfreq,sizeof(intfreq));
  frequency = intfreq;

  fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_LNA_GAIN,&lna_gain,sizeof(lna_gain));
  fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_MIXER_GAIN,&mixer_gain,sizeof(mixer_gain));
  fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_IF_GAIN1,&if_gain,sizeof(if_gain));

  pthread_mutex_unlock(&FCD.mutex);

  while(1){

    if(No_hold_open){
      fcdClose(FCD.phd);
      FCD.phd = NULL;
    }
    pthread_mutex_lock(&FCD.mutex);
    while(!FCD.pending)
      pthread_cond_wait(&FCD.cond,&FCD.mutex);
    FCD.pending = 0;

    pthread_mutex_unlock(&FCD.mutex); // Too early!!

    if(FCD.phd == NULL){
      if((FCD.phd = fcdOpen(FCD.sdr_name,sizeof(FCD.sdr_name),Dongle)) == NULL){
	fprintf(stderr,"funcube: can't re-open control port, aborting\n");
	abort();
      }
    }      

    // See what has changed
    if(FCD.status.lna_gain != lna_gain){
      lna_gain = FCD.status.lna_gain;
#if 0
      fprintf(stderr,"lna %s\n",lna_gain ? "ON" : "OFF");
#endif
      fcdAppSetParam(FCD.phd,FCD_CMD_APP_SET_LNA_GAIN,&lna_gain,sizeof(lna_gain));
    }
    if(FCD.status.mixer_gain != mixer_gain){
      mixer_gain = FCD.status.mixer_gain;
#if 0
      fprintf(stderr,"mixer %s\n",mixer_gain ? "ON" : "OFF");
#endif
      fcdAppSetParam(FCD.phd,FCD_CMD_APP_SET_MIXER_GAIN,&mixer_gain,sizeof(mixer_gain));
    }
    if(FCD.status.if_gain != if_gain){
      if_gain = FCD.status.if_gain;
#if 0
      fprintf(stderr,"IF gain %d db\n",if_gain);
#endif
      fcdAppSetParam(FCD.phd,FCD_CMD_APP_SET_IF_GAIN1,&if_gain,sizeof(if_gain));
    }
    if(FCD.status.frequency != frequency){
      intfreq = frequency = FCD.status.frequency;
      fcdAppSetFreq(FCD.phd,intfreq);
    }
#if 0
    // TEST
    fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_LNA_GAIN,&lna_gain,sizeof(lna_gain));
    fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_MIXER_GAIN,&mixer_gain,sizeof(mixer_gain));
    fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_IF_GAIN1,&if_gain,sizeof(if_gain));
    fprintf(stderr,"fcd readback: lna_gain %d mixer_gain %d if_gain %d\n",lna_gain,mixer_gain,if_gain);
#endif
  }
}

// Status display thread
void *display(void *arg){
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


