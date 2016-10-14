// $Id: funcube.c,v 1.3 2016/10/14 00:53:34 karn Exp karn $
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
#include <signal.h>
#include <locale.h>

#include "fcd.h"
#include "sdr.h"
#include "command.h"
#include "dsp.h"

#define MAXPKT 1500

void *display(void *arg);
pthread_t Display_thread;


struct sdrstate {
  // Stuff for sending commands
  void *phd;                 // Opaque pointer to type hid_device
  pthread_mutex_t mutex; // Allow only one command to the device at a time
  pthread_cond_t cond;
  int pending;               // set to ask task to update dongle

  // Analog gain settings
  int max_gain;
  int lna_gr,mixer_gr,if_gr; // Current analog gain settings inside device
  int agc_holdoff;
  int agc_holdoff_count;

  // First (analog) LO synthesizer
  int intfreq;               // Apparent first LO frequency, integer Hz, without correction; protect with mutex
  double calibrate;          // Multiply desired frequency by (1 + calibrate) before setting
  double first_LO;           // true hardware LO frequency, Hz - note hardware works in integers

  // Digital signal statistics
  complex float DC_offset;
  complex float energy;

  double sinphi;              // smoothed sine of phase error ~ phase error in radians for small error
  float sig_high,sig_low;     // energy set points for adjusting analog AGC

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
int Gain = 0;

void *fcd_command(void *arg);
int process_fc_command(char *,int);
double set_fc_LO(double);

int main(int argc,char *argv[]){
  struct sockaddr_in6 address;
  char *locale;
  int c,ctl;
  int ctl_port = 4160;
  int blocksize = 4096;
  double f = 147435000;
  int dongle = 0;
  
  locale = getenv("LANG");
  while((c = getopt(argc,argv,"g:d:vf:p:l:b:")) != EOF){
    switch(c){
    case 'g':
      Gain = atoi(optarg);
      break;
    case 'd':
      dongle = atoi(optarg);
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
  if(Verbose)
    fprintf(stderr,"funcube dongle %d: min gain reduction %d dB, blocksize %d, UDP control port %d\n",
	    dongle,Gain,blocksize,ctl_port);
  setlocale(LC_ALL,locale);
  signal(SIGPIPE,SIG_IGN);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  
  address.sin6_family = AF_INET6;
  address.sin6_port = htons(ctl_port);
  address.sin6_flowinfo = 0;
  address.sin6_addr = in6addr_any;
  address.sin6_scope_id = 0;
  
  if((ctl = socket(PF_INET6,SOCK_DGRAM, 0)) == -1){
    fprintf(stderr,"funcube: can't open control socket\n");
    exit(1);
  }
  if(bind(ctl,&address,sizeof(address)) != 0){
    fprintf(stderr,"funcube: bind failed\n");
    exit(1);
  }
  fcntl(ctl,F_SETFL,O_NONBLOCK);
  front_end_init(dongle,ADC_samprate,blocksize);
  usleep(100000);
  set_fc_LO(f); // Must be done after init

  pthread_mutex_lock(&FCD.mutex);
  mirics_gain(FCD.first_LO,90,&FCD.if_gr,&FCD.lna_gr,&FCD.mixer_gr); // start low
  FCD.pending = 1;
  pthread_cond_broadcast(&FCD.cond);
  pthread_mutex_unlock(&FCD.mutex);    
  
  if(Verbose > 1)
    pthread_create(&Display_thread,NULL,display,NULL);

  while(1){
    complex float sampbuf[blocksize];
    socklen_t addrlen;
    char pktbuf[MAXPKT];
    int rdlen;
    
    while(addrlen = sizeof(address), (rdlen = recvfrom(ctl,&pktbuf,sizeof(pktbuf),0,&address,&addrlen)) > 0){
      // Commands have priority - is this right?
      process_fc_command(pktbuf,rdlen);
    }
    get_adc(sampbuf,blocksize);
    // We've ignored SIGPIPE, so if the reader has gone away this write will fail with EPIPE. Just keep going.
    write(1,sampbuf,blocksize*sizeof(*sampbuf));
  }
  close(ctl);
  exit(0);
}


int front_end_init(int dongle, int samprate,int L){
  int r;
  unsigned int exact_rate;
  snd_pcm_uframes_t buffer_size;

  FCD.samprate = samprate;
  FCD.sig_high = -15;
  FCD.sig_low = -35;
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

// Read buffer of samples from front end, correct for DC offsets and I/Q gain
float get_adc(complex float *buffer,int L){
  short samps[2*L];
  int r,n;
  float i_gain,gain;
  complex float residual_DC,energy,dot;
  double alpha,sinphi,cosphi,crosstalk,boost;
  float energy_db;

  // Read block of I/Q samples from A/D converter
  do {
    snd_pcm_state_t state;
    state = snd_pcm_state(FCD.sdr_handle);
    if(state != SND_PCM_STATE_RUNNING && state != SND_PCM_STATE_PREPARED){
      FCD.overrun++;
      snd_pcm_prepare(FCD.sdr_handle);
    }
    if((r = snd_pcm_readi(FCD.sdr_handle,samps,L)) < 0){
      fprintf(stderr,"funcube read error %s\n",snd_strerror(r));
      usleep(500000); // Just to keep from locking things up
    }
  } while(r != L);


  gain = dB2voltage(FCD.lna_gr + FCD.mixer_gr + FCD.if_gr - FCD.max_gain);
  // Adjust current block with smoothed values
  alpha = sqrt(crealf(FCD.energy)/cimagf(FCD.energy));
  i_gain = 1/alpha;
  sinphi = FCD.sinphi;
  cosphi = sqrt(1 - sinphi*sinphi);
  crosstalk = -sinphi/(alpha*cosphi);
  boost = 1/cosphi;

  // Compute DC component, I/Q energies and I/Q dot products for
  // DC removal, I/Q gain balancing and I/Q phase balancing
  residual_DC = 0;
  energy = 0;
  dot = 0;

  for(n=0;n<L;n++){
    buffer[n] = CMPLXF(samps[2*n],samps[2*n+1]) - FCD.DC_offset;
    residual_DC += buffer[n];
    buffer[n] *= (1./SHRT_MAX);
    energy += CMPLXF(crealf(buffer[n])*crealf(buffer[n]), cimagf(buffer[n])*cimagf(buffer[n]));
    dot += crealf(buffer[n]) * cimagf(buffer[n]);
    buffer[n] = gain * CMPLXF(i_gain * crealf(buffer[n]),
			      crosstalk * crealf(buffer[n]) + boost * cimagf(buffer[n]));
  }
  // Update stats except after a gain change
  residual_DC /= L;
  energy /= L;
  dot /= L;
  if(FCD.agc_holdoff_count != 0)
    FCD.agc_holdoff_count--;
  if(FCD.agc_holdoff_count == 0){  // Clamp estimates after an AGC adjustment
    int gr;

    FCD.DC_offset += 0.05 * residual_DC;
    FCD.energy = 0.95 * FCD.energy + 0.05 * energy; // this will give us a smoothed imbalance
    alpha = sqrt(crealf(energy)/cimagf(energy)); // I/Q gain imbalance
    i_gain = 1/alpha;
    sinphi = i_gain * dot/cimagf(energy); // sin(phase error) Can this ever be < -1 or > +1?
    FCD.sinphi = 0.95 * FCD.sinphi + 0.05 * sinphi; // keep smoothed value
    energy_db = power2dB(crealf(energy) + cimagf(energy));
    gr = FCD.lna_gr + FCD.mixer_gr + FCD.if_gr; // current gain reduction
    if(energy_db > FCD.sig_high || (gr > Gain && energy_db < FCD.sig_low)){
      // Adjust analog gains
      int adjust = energy_db - (FCD.sig_high + FCD.sig_low)/2;
      if(gr + adjust <= Gain)
	gr = Gain; // minimum gain reduction
      else
	gr += adjust;

      pthread_mutex_lock(&FCD.mutex);
      mirics_gain(FCD.first_LO,gr,&FCD.if_gr,&FCD.lna_gr,&FCD.mixer_gr);
      FCD.pending = 1;
      pthread_cond_broadcast(&FCD.cond);
      pthread_mutex_unlock(&FCD.mutex);    
      FCD.agc_holdoff_count = FCD.agc_holdoff; // Start holdoff timer
#if 0
      fprintf(stderr,"funcube energy %.1f dB gain reduction %d db: if %d lna %d mixer %d\n",
	      energy_db,gr,FCD.if_gr,FCD.lna_gr,FCD.mixer_gr);
#endif
    }
  }
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


// Tune hardware in integer steps as close to f as possible, return actual frequency
// Accounts for calibration offset, so result will probably have a fractional hertz
// This fraction can be allowed for in the second LO (the one in software)
double set_fc_LO(double f){
  if(f < 60e6){  // AM/LW/MW/SW
    FCD.max_gain = 96;
  } else if(f < 120e6){ // VHF band II
    FCD.max_gain = 106;
  } else if(f < 250e6){ // VHF band III
    FCD.max_gain = 107;
  } else if(f < 420e6){ // ?
    FCD.max_gain = 107; // Does it even cover this range?
  } else if(f < 1e9){   // Band IV/V
    FCD.max_gain = 95.8;
  } else {
    FCD.max_gain = 106;
  }


  pthread_mutex_lock(&FCD.mutex);
  FCD.pending = 1;
  FCD.intfreq = (int)f;
  pthread_cond_broadcast(&FCD.cond);
  pthread_mutex_unlock(&FCD.mutex);    

  FCD.first_LO = f;
  return FCD.first_LO;
}

void *fcd_command(void *arg){
  unsigned char lna_gr,mixer_gr,if_gr;
  int intfreq;
  unsigned char val;

  // Load current tuner state
  pthread_mutex_lock(&FCD.mutex);
  fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_FREQ_HZ,(unsigned char *)&FCD.intfreq,sizeof(FCD.intfreq));
  intfreq = FCD.intfreq;
  FCD.first_LO = FCD.intfreq / (1 + FCD.calibrate); // store true frequency

  fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_LNA_GAIN,&val,sizeof(val));
  if(val == 0 && intfreq < 1e9)
    lna_gr = 24;
  else if(val == 0)
    lna_gr = 7;
  else
    lna_gr = 0;

  fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_MIXER_GAIN,&val,sizeof(val));
  if(val == 0)
    mixer_gr = 19;
  else
    mixer_gr = 0;

  fcdAppGetParam(FCD.phd,FCD_CMD_APP_GET_IF_GAIN1,&val,sizeof(val));
  if_gr = 59 - val;

  pthread_mutex_unlock(&FCD.mutex);

  // For each changed parameter, first just set a flag so we can release the mutex quickly
  // The fcdApp calls can stall, and we don't want to delay our client unnecessarily
  while(1){
    int set_lna,set_mixer,set_gain,set_freq;

    set_lna = set_mixer = set_gain = set_freq = 0;
    pthread_mutex_lock(&FCD.mutex);
    while(!FCD.pending)
      pthread_cond_wait(&FCD.cond,&FCD.mutex);
    FCD.pending = 0;

    // See what has changed
    if(FCD.lna_gr != lna_gr){
      lna_gr = FCD.lna_gr;
      set_lna++;
    }
    if(FCD.mixer_gr != mixer_gr){
      mixer_gr = FCD.mixer_gr;
      set_mixer++;
    }
    if(FCD.if_gr != if_gr){
      if_gr = FCD.if_gr;
      set_gain++;
    }
    if(FCD.intfreq != intfreq){
      intfreq = FCD.intfreq;
      set_freq++;
    }
    pthread_mutex_unlock(&FCD.mutex);

    if(set_lna){
      val = lna_gr != 0 ? 0 : 1;
#if 0
      fprintf(stderr,"lna %s\n",val ? "ON" : "OFF");
#endif
      fcdAppSetParam(FCD.phd,FCD_CMD_APP_SET_LNA_GAIN,&val,sizeof(val));
    }
    if(set_mixer){
      val = mixer_gr != 0 ? 0 : 1;
#if 0
      fprintf(stderr,"mixer %s\n",val ? "ON" : "OFF");
#endif
      fcdAppSetParam(FCD.phd,FCD_CMD_APP_SET_MIXER_GAIN,&val,sizeof(val));
    }
    if(set_gain){
      val = 59 - if_gr;
#if 0
      fprintf(stderr,"IF gain %d db\n",val);
#endif
      fcdAppSetParam(FCD.phd,FCD_CMD_APP_SET_IF_GAIN1,&val,sizeof(val));
    }
    if(set_freq)
      fcdAppSetFreq(FCD.phd,intfreq);
  }
}

// Status display thread
void *display(void *arg){
  float powerdB;

  fprintf(stderr,"Frequency                     DC Offset       I/Q imbal   LNA  Mix    IF  Gain  A/D  Signal\n");
  fprintf(stderr,"Hz                            I       Q       rad    dB    dB   dB    dB  dB   dBFS      dB\n");

  while(1){
    powerdB = power2dB(crealf(FCD.energy) + cimagf(FCD.energy));

    fprintf(stderr,"%'-15.0lf%'16.2f%'8.2f%'10.5f%'6.1f   %3d  %3d%'6d%'4d %'6.1f%'8.1f\r",
	    FCD.first_LO,
	    crealf(FCD.DC_offset),cimagf(FCD.DC_offset),
	    FCD.sinphi,
	    power2dB(crealf(FCD.energy)/cimagf(FCD.energy)),
	    -FCD.lna_gr,-FCD.mixer_gr,
	    -FCD.if_gr,
	    FCD.max_gain - (FCD.lna_gr+FCD.mixer_gr+FCD.if_gr),
	    powerdB,
	    powerdB + FCD.lna_gr + FCD.mixer_gr + FCD.if_gr - FCD.max_gain);
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


