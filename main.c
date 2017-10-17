// $Id: main.c,v 1.88 2017/10/16 13:22:29 karn Exp karn $
// Read complex float samples from multicast stream (e.g., from funcube.c)
// downconvert, filter, demodulate, optionally compress and multicast audio
// Copyright 2017, Phil Karn, KA9Q, karn@ka9q.net
#define _GNU_SOURCE 1
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#if defined(linux)
#include <bsd/string.h>
#endif
#include <math.h>
#include <complex.h>
#undef I
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <locale.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <netdb.h>
#include <errno.h>

#include "radio.h"
#include "filter.h"
#include "dsp.h"
#include "audio.h"
#include "rtp.h"
#include "multicast.h"

#define MAXPKT 1500 // Maximum bytes of data in incoming I/Q packet

void closedown(int);
void *input_loop(void *);

pthread_t Display_thread;

// Primary control blocks for downconvert/filter/demodulate and audio output
// Note: initialized to all zeroes, like all global variables
struct demod Demod;
struct audio Audio;

// Multicast receive statistics
int Skips;
int Delayed;


// Parameters with default values
char Libdir[] = "/usr/local/share/ka9q-radio";
int Nthreads = 1;
int DAC_samprate = 48000;
int Quiet = 0;
int Verbose = 0;
char Statepath[PATH_MAX];
char Locale[256] = "en_US.UTF-8";
int Update_interval = 100;  // 100 ms between screen updates
// SDR alias keep-out region, i.e., stay between -(samprate/2 - IF_EXCLUDE) and (samprate/2 - IF_EXCLUDE)
int IF_EXCLUDE = 16000; // Hardwired for UK Funcube Dongle Pro+, make this more general


int main(int argc,char *argv[]){
  // if we have root, up our priority and drop privileges
  int prio = getpriority(PRIO_PROCESS,0);
  prio = setpriority(PRIO_PROCESS,0,prio - 5);

  // Drop root if we have it
  seteuid(getuid());

  // Set up program defaults
  // Some can be overridden by state file or command line args
  {
    // The display thread assumes en_US.UTF-8, or anything with a thousands grouping character
    // Otherwise the cursor movements will be wrong
    char const * const cp = getenv("LANG");
    if(cp != NULL){
      strlcpy(Locale,cp,sizeof(Locale));
    }
  }
  setlocale(LC_ALL,Locale); // Set either the hardwired default or the value of $LANG if it exists
  snprintf(Statepath,sizeof(Statepath),"%s/%s",getenv("HOME"),".radiostate");
  Statepath[sizeof(Statepath)-1] = '\0';

  if(readmodes("modes.txt") != 0){
    fprintf(stderr,"Can't read mode table\n");
    exit(1);
  }

  // Must do this before first filter is created with set_mode(), otherwise a segfault can occur
  fftwf_import_system_wisdom();

  struct demod * const demod = &Demod; // Only one demodulator per program for now

  // Set program defaults, can be overridden by state file and command line args, in that order
  memset(demod,0,sizeof(*demod));
  demod->audio = &Audio; // Link to audio output system
  demod->audio->samprate = DAC_samprate; // currently 48 kHz, hard to change
  strcpy(demod->mode,"FM");
  demod->start_freq = 147.435e6;  // LA "animal house" repeater, active all night for testing

  demod->L = 4096;      // Number of samples in buffer: FFT length = L + M - 1
  demod->M = 4096+1;    // Length of filter impulse response
  demod->kaiser_beta = 3.0; // Reasonable compromise
  strlcpy(demod->iq_mcast_address_text,"239.1.2.1",sizeof(demod->iq_mcast_address_text));
  demod->headroom = pow(10.,-15./20); // -15 dB
  demod->audio->opus_bitrate = 0;  // off by default
  demod->audio->opus_blocktime = 20;   // 20 ms
  strlcpy(demod->audio->audio_mcast_address_text,"239.2.1.1",sizeof(demod->audio->audio_mcast_address_text));
  demod->tunestep = 0;  // single digit hertz position
  demod->calibrate = 0;
  demod->imbalance = 1; // 0 dB

  // set invalid to start
  demod->input_source_address.sa_family = -1; // Set invalid
  demod->low = NAN;
  demod->high = NAN;
  set_shift(demod,0);

  // Find any file argument and load it
  char optstring[] = "a:B:c:d:f:I:k:l:L:m:M:pr:R:qo:s:t:u:vx";
  while(getopt(argc,argv,optstring) != EOF)
    ;
  if(argc > optind)
    loadstate(demod,argv[optind]);
  else
    loadstate(demod,"default");
  
  // Go back and re-read args for real this time, possibly overwriting loaded parameters
  optind = 1;
  int c;
  while((c = getopt(argc,argv,optstring)) != EOF){
    switch(c){
    case 'a':
      memcpy(Audio.localdev,optarg,sizeof(Audio.localdev));
      break;
    case 'B':   // Opus encoder block time; must be 2.5, 5, 10, 20, 40, 60, 80, 120
      Audio.opus_blocktime = strtod(optarg,NULL);
      break;
    case 'c':   // SDR TCXO and A/D clock calibration in parts per million
      set_cal(demod,1e-6*strtod(optarg,NULL));
      break;
    case 'd':
      demod->doppler_command = optarg;
      break;
    case 'f':   // Initial RF tuning frequency
      demod->start_freq = parse_frequency(optarg);
      break;
    case 'I':   // Multicast address to listen to for I/Q data
      strlcpy(demod->iq_mcast_address_text,optarg,sizeof(demod->iq_mcast_address_text));
      break;
    case 'k':   // Kaiser window shape parameter; 0 = rectangular
      demod->kaiser_beta = strtod(optarg,NULL);
      break;
    case 'l':   // Locale, mainly for numerical output format
      strlcpy(Locale,optarg,sizeof(Locale));
      setlocale(LC_ALL,Locale);
      break;
    case 'L':   // Pre-detection filter block size
      demod->L = strtol(optarg,NULL,0);
      break;
    case 'm':   // receiver mode (AM/FM, etc)
      strlcpy(demod->mode,optarg,sizeof(demod->mode));
      break;
    case 'M':   // Pre-detection filter impulse length
      demod->M = strtol(optarg,NULL,0);
      break;
    case 'o':   // Opus codec compressed bit rate target
      Audio.opus_bitrate = strtol(optarg,NULL,0);
      if(Audio.opus_bitrate < 1000)
	Audio.opus_bitrate *= 1000;	// Interpret as kilobits/sec
      break;
    case 'p':
      Audio.rtp_pcm = 1;
      break;
    case 'q':
      Quiet++;  // Suppress display
      break;
    case 'R':   // Set audio multicast address
      if((strncmp(optarg,"/",1) == 0) || (strncmp(optarg,"./",2) == 0)){
	Audio.filename = optarg;
	if((Audio.stream = fopen(Audio.filename,"w")) == NULL){
	  fprintf(stderr,"Can't stream to %s\n",Audio.filename);
	  exit(1);
	}
      } else 
	strlcpy(Audio.audio_mcast_address_text,optarg,sizeof(Audio.audio_mcast_address_text));
      break;
    case 's':
      {
	double shift = strtod(optarg,NULL);
	set_shift(demod,shift);
      }
      break;
    case 't':   // # of threads to use in FFTW3
      Nthreads = strtol(optarg,NULL,0);
      fftwf_init_threads();
      fftwf_plan_with_nthreads(Nthreads);
      fprintf(stderr,"Using %d threads for FFTs\n",Nthreads);
      break;
    case 'u':   // Display update rate
      Update_interval = strtol(optarg,NULL,0);
      break;
    case 'v':   // Extra debugging
      Verbose++;
      break;
    case 'x':   // Enable Opus discontinuous mode (can cause problems with CW at very high SNR)
      Audio.opus_dtx = 1;
      break;
    default:
      fprintf(stderr,"Usage: %s [-a audiodev] [-B opus_blocktime] [-c calibrate_ppm] [-d doppler_command] [-f frequency] [-I iq multicast address] [-k kaiser_beta] [-l locale] [-L blocksize] [-m mode] [-M FIRlength] [-o opus_bitrate] [-p] [-q] [-R Audio multicast address] [-s shift offset] [-t threads] [-u update_ms] [-v] [-x]\n",argv[0]);
      exit(1);
      break;
    }
  }
  fprintf(stderr,"General coverage receiver for the Funcube Pro and Pro+\n");
  fprintf(stderr,"Copyright 2017 by Phil Karn, KA9Q; may be used under the terms of the GNU General Public License\n");
  
  // Set up actual demod state
  demod->ctl_fd = -1;   // Invalid
  demod->input_fd = -1; // Invalid
  demod->write_ptr = 0;

#if !defined(NDEBUG)
  // Detect early starts
  demod->second_LO_phasor = NAN;
  demod->second_LO_phasor_step = NAN;
#endif

  // Circular buffer between input thread and demodulator thread
  demod->corr_data = malloc(DATASIZE * sizeof(*demod->corr_data));
  
  pthread_mutex_init(&demod->status_mutex,NULL);
  pthread_cond_init(&demod->status_cond,NULL);
  pthread_mutex_init(&demod->data_mutex,NULL);
  pthread_cond_init(&demod->data_cond,NULL);
  pthread_mutex_init(&demod->doppler_mutex,NULL);
  pthread_mutex_init(&demod->shift_mutex,NULL);
  pthread_mutex_init(&demod->second_LO_mutex,NULL);

  // Input socket for I/Q data from SDR
  demod->input_fd = setup_mcast(demod->iq_mcast_address_text,0);
  if(demod->input_fd == -1){
    fprintf(stderr,"Can't set up I/Q input\n");
    exit(1);
  }
  // For sending commands to front end
  if((demod->ctl_fd = socket(PF_INET,SOCK_DGRAM, 0)) == -1)
    perror("can't open control socket");

  if(setup_audio(demod->audio) != 0){
    fprintf(stderr,"Audio setup failed\n");
    exit(1);
  }

  // The input thread must run before calling these next functions, otherwise they'll deadlock
  pthread_create(&demod->input_thread,NULL,input_loop,demod);

  set_second_LO(demod,0); // Initialize LO2 phasor so demod task won't fail at startup

  // Optional doppler correction
  if(demod->doppler_command)
    pthread_create(&demod->doppler_thread,NULL,doppler,demod);

  // Actually set the mode and frequency already specified
  // These wait until the SDR sample rate is known, so they'll block if the SDR isn't running
  fprintf(stderr,"Waiting for SDR response...\n");
  set_freq(demod,demod->start_freq,NAN); 
  demod->start_freq = 0;
  demod->gain = dB2voltage(30.); // Empirical starting value
  set_mode(demod,demod->mode,0); // Don't override with defaults from mode table 

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  signal(SIGPIPE,SIG_IGN);

  if(!Quiet)
    pthread_create(&Display_thread,NULL,display,demod);

  while(1)
    usleep(1000000); // probably get rid of this

  exit(0);
}

void display_cleanup(void *);
void audio_cleanup(void *);

void closedown(int a){
  if(!Quiet)
    fprintf(stderr,"radio: caught signal %d: %s\n",a,strsignal(a));
  audio_cleanup(NULL);
  display_cleanup(NULL);
  exit(1);
}

// Read from RTP network socket, remove DC offsets,
// fix I/Q gain and phase imbalance,
// Write corrected data to circular buffer, wake up demodulator thread(s)
// when data is available and when SDR status (frequency, sampling rate) changes
void *input_loop(void *arg){
  pthread_setname("input");
  assert(arg != NULL);
  struct demod * const demod = arg;
		 
  struct status new_status;
  int16_t samples[MAXPKT];
  struct rtp_header rtp;
  struct iovec iovec[3];

  // Packet consists of Ethernet, IP and UDP header (already stripped)
  // then standard Real Time Protocol (RTP), a status header and the PCM
  // I/Q data. RTP is an IETF standard, so it uses big endian numbers
  // The status header and I/Q data are *not* standard, so we save time
  // by using machine byte order (almost certainly little endian).
  // Note this is a portability problem if this system and the one generating
  // the data have opposite byte orders. But who's big endian anymore?
  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = &new_status;
  iovec[1].iov_len = sizeof(new_status);
  iovec[2].iov_base = samples;
  iovec[2].iov_len = sizeof(samples);
  
  struct msghdr message;
  message.msg_name = &demod->input_source_address;
  message.msg_namelen = sizeof(demod->input_source_address);
  message.msg_iov = iovec;
  message.msg_iovlen = sizeof(iovec) / sizeof(struct iovec);
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  int eseq = -1;

  while(1){
    // Listen for an I/Q packet
    fd_set mask,errmask;
    FD_ZERO(&mask);
    FD_ZERO(&errmask);
    FD_SET(demod->input_fd,&mask);
    FD_SET(demod->input_fd,&errmask);

    // The timeout recovers us if demod->input_fd changes, e.g., from the interactive 'I' command in display.c
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    select(FD_SETSIZE,&mask,NULL,&errmask,&timeout);

    if(FD_ISSET(demod->input_fd,&errmask))
      break;

    if(FD_ISSET(demod->input_fd,&mask)){
      // Receive I/Q data from front end
      int cnt = recvmsg(demod->input_fd,&message,0);
      if(cnt == -1){
	if(errno != EINTR) // probably happens routinely
	  perror("recvfrom");
	break;
      }
      if(cnt < sizeof(rtp) + sizeof(demod->status))
	continue; // Too small, ignore
      
      // Convert RTP header to host byte order
      demod->iq_packets++;
      rtp.ssrc = ntohl(rtp.ssrc);
      rtp.seq = ntohs(rtp.seq);
      rtp.timestamp = ntohl(rtp.timestamp);
      if(eseq != -1){
	if((int16_t)(eseq - rtp.seq) < 0){
	  Skips++;
	} else if((int16_t)(eseq - rtp.seq) > 0){
	  Delayed++;
	}
      }
      eseq = rtp.seq + 1;

      if(memcmp(&demod->status,&new_status,sizeof(demod->status)) != 0){
	// Protect status with a mutex and signal a condition when it changes
	// since demod threads will be waiting for this
	pthread_mutex_lock(&demod->status_mutex);
	memcpy(&demod->status,&new_status,sizeof(demod->status));
	// We now know the A/D sample rate
	// These need to be set before the demod thread starts!
	// Signalled every time the status is updated
	// status.samprate contains *nominal* A/D sample rate
	// demod->samprate contains *corrected* A/D sample rate
	// Use nominal rates here so result is clean integer
	demod->decimate = 1; demod->interpolate = 1;
	if(demod->status.samprate >= Audio.samprate){
	  // Sample rate is higher than audio rate; decimate
	  demod->decimate = demod->status.samprate / Audio.samprate;
	  demod->samprate = demod->status.samprate * (1 + demod->calibrate);
	  demod->max_IF = demod->status.samprate/2 - IF_EXCLUDE;
	  demod->min_IF = -demod->max_IF;
	} else {
	  // Sample rate is lower than audio rate
	  // Interpolate up to audio rate, pretend sample rate is audio rate
	  demod->interpolate = Audio.samprate / demod->status.samprate;	  
	  demod->samprate = Audio.samprate * (1 + demod->calibrate);
	  demod->max_IF = Audio.samprate/2 - IF_EXCLUDE;
	  demod->min_IF = -demod->max_IF;
	}
	// re-call these two to recalculate their phasor steps in case the sample rate
	// has changed or was unknown
	set_second_LO(demod,get_second_LO(demod));
	set_shift(demod,demod->shift);
	pthread_cond_broadcast(&demod->status_cond);
	pthread_mutex_unlock(&demod->status_mutex);
      }
      // Pass PCM I/Q samples to corrector and input queue
      cnt -= sizeof(rtp) + sizeof(demod->status);
      // count 4-byte stereo samples
      proc_samples(demod,samples,cnt/4);
    }
  }
  return NULL;
}
 
// Load calibration factor for specified sending IP
int loadcal(struct demod *demod){
  char source[NI_MAXHOST];
  char sport[NI_MAXSERV];
  memset(source,0,sizeof(source));
  memset(sport,0,sizeof(sport));
  // Turn into printable IP address string (no DNS resolution)
  if(getnameinfo((struct sockaddr *)&demod->input_source_address,sizeof(demod->input_source_address),
		 source,sizeof(source),
		 sport,sizeof(sport),NI_NOFQDN|NI_DGRAM|NI_NUMERICHOST) != 0){
    return -1;    // failed
  }

  FILE *fp;
  char pathname[PATH_MAX];
  snprintf(pathname,sizeof(pathname),"%s/calibrate-%s",Statepath,source);

  if((fp = fopen(pathname,"r")) == NULL){
    fprintf(stderr,"Can't read calibration file %s\n",pathname);
    return -1;
  }
  double calibrate;
  if(fscanf(fp,"%lg",&calibrate) == 1){
    set_cal(demod,calibrate);
  }
  fclose(fp);
  return 0;
}
// Save calibration factor for specified sending IP
int savecal(struct demod *demod){
  // Dump receiver state to file
  char source[NI_MAXHOST];
  char sport[NI_MAXSERV];
  memset(source,0,sizeof(source));
  memset(sport,0,sizeof(sport));
  // Turn into printable IP address string (no DNS resolution)
  if(getnameinfo((struct sockaddr *)&demod->input_source_address,sizeof(demod->input_source_address),
		 source,sizeof(source),
		 sport,sizeof(sport),NI_NOFQDN|NI_DGRAM|NI_NUMERICHOST) != 0){
    return -1; // failed
  }
    

  FILE *fp;
  char pathname[PATH_MAX];
  snprintf(pathname,sizeof(pathname),"%s/calibrate-%s",Statepath,source);

  if((fp = fopen(pathname,"w")) == NULL){
    fprintf(stderr,"Can't write calibration file %s\n",pathname);
    return -1;
  }
  fprintf(fp,"%lg\n",demod->calibrate);
  fclose(fp);
  return 0;
}


// Save receiver state to file
// Path is Statepath[] = $HOME/.radiostate
int savestate(struct demod *dp,char const *filename){
  // Dump receiver state to file
  FILE *fp;
  char pathname[PATH_MAX];
  if(filename[0] == '/')
    strlcpy(pathname,filename,sizeof(pathname));    // Absolute path
  else
    snprintf(pathname,sizeof(pathname),"%s/%s",Statepath,filename);

  if((fp = fopen(pathname,"w")) == NULL){
    fprintf(stderr,"Can't write state file %s\n",pathname);
    return -1;
  }
  fprintf(fp,"#KA9Q DSP Receiver State dump\n");
  fprintf(fp,"Locale %s\n",Locale);
  fprintf(fp,"Source %s\n",dp->iq_mcast_address_text);
  if(dp->audio){
    fprintf(fp,"Audio output %s\n",dp->audio->audio_mcast_address_text);
    fprintf(fp,"Opus blocktime %.0f\n",dp->audio->opus_blocktime);
    fprintf(fp,"OPUS bitrate %d\n",dp->audio->opus_bitrate);
  }
  fprintf(fp,"Blocksize %d\n",dp->L);
  fprintf(fp,"Impulse len %d\n",dp->M);
  fprintf(fp,"Frequency %.3f Hz\n",get_freq(dp));
  fprintf(fp,"Mode %s\n",dp->mode);
  fprintf(fp,"Shift %.3f Hz\n",dp->shift);
  fprintf(fp,"Filter low %.3f Hz\n",dp->low);
  fprintf(fp,"Filter high %.3f Hz\n",dp->high);
  fprintf(fp,"Tunestep %d\n",dp->tunestep);
  fclose(fp);
  return 0;
}
// Load receiver state from file
// Some of these are problematic since they're overwritten from the mode
// table when the mode is actually set on the first A/D packet:
// shift, filter low, filter high, tuning step (not currently set)
// 
int loadstate(struct demod *dp,char const *filename){
  FILE *fp;
  char pathname[PATH_MAX];
  if(filename[0] == '/')
    strlcpy(pathname,filename,sizeof(pathname));
  else
    snprintf(pathname,sizeof(pathname),"%s/%s",Statepath,filename);

  if((fp = fopen(pathname,"r")) == NULL){
    fprintf(stderr,"Can't read state file %s\n",pathname);
    return -1;
  }
  char line[PATH_MAX];
  while(fgets(line,sizeof(line),fp) != NULL){
    chomp(line);
    if(sscanf(line,"Frequency %lf",&dp->start_freq) > 0){
    } else if(strncmp(line,"Mode ",5) == 0){
      strlcpy(dp->mode,&line[5],sizeof(dp->mode));
    } else if(sscanf(line,"Shift %lf",&dp->shift) > 0){
    } else if(sscanf(line,"Filter low %f",&dp->low) > 0){
    } else if(sscanf(line,"Filter high %f",&dp->high) > 0){
    } else if(sscanf(line,"Kaiser Beta %f",&dp->kaiser_beta) > 0){
    } else if(sscanf(line,"Blocksize %d",&dp->L) > 0){
    } else if(sscanf(line,"Impulse len %d",&dp->M) > 0){
    } else if(sscanf(line,"Tunestep %d",&dp->tunestep) > 0){
    } else if(sscanf(line,"Source %256s",dp->iq_mcast_address_text) > 0){
      // Array sizes defined elsewhere!
    } else if(dp->audio && sscanf(line,"Audio output %256s",dp->audio->audio_mcast_address_text) > 0){
    } else if(dp->audio && sscanf(line,"Opus blocktime %f",&dp->audio->opus_blocktime) > 0){
    } else if(dp->audio && sscanf(line,"OPUS bitrate %d",&dp->audio->opus_bitrate) > 0){
    } else if(sscanf(line,"Locale %256s",Locale)){
      setlocale(LC_ALL,Locale);
    }
  }
  fclose(fp);
  return 0;
}
