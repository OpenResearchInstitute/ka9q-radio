// $Id: main.c,v 1.53 2017/07/29 23:55:54 karn Exp karn $
// Read complex float samples from stdin (e.g., from funcube.c)
// downconvert, filter and demodulate
// Take commands from UDP socket
#define _GNU_SOURCE 1
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#undef I
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <locale.h>
#include <signal.h>
#include <sys/select.h>


#include "radio.h"
#include "filter.h"
#include "dsp.h"
#include "audio.h"
#include "rtp.h"
#include "multicast.h"

#define MAXPKT 1500

void closedown(int);
void *input_loop(struct demod *);
int process_command(struct demod *,char const *,int);

pthread_t Display_thread;
struct demod Demod;
struct sockaddr_in Ctl_address;
struct sockaddr_in Input_source_address;
struct sockaddr_in BB_mcast_sockaddr;
int Input_fd = -1;
int Ctl_fd = -1;
int Demod_sock = -1;
// Multicast receive statistics
int Skips;
int Delayed;


// Parameters with default values
char Libdir[] = "/usr/local/share/ka9q-radio";
int Nthreads = 1;
int ADC_samprate = 192000;
int DAC_samprate = 48000;
int Quiet = 0;
int Verbose = 0;
int Ctl_port = 4159;
char IQ_mcast_address_text[256] = "239.1.2.1"; // Long enough for IPv4, but what about IPv6?
char Mcast_dest_port[256] = "5004";
char BB_mcast_address_text[256] = "239.2.1.1"; 
// We have to hold the requested startup frequency until we know the IP address
// of the SDR front end to send it to
double Startup_freq = 0;
enum mode Start_mode = FM;
char Locale[256] = "en_US.UTF-8";


int main(int argc,char *argv[]){
  struct demod * const demod = &Demod;

  {
    // The display thread assumes en_US.UTF-8, or anything with a thousands grouping character
    // Otherwise the cursor movements will be wrong
    char const * const cp = getenv("LANG");
    if(cp != NULL){
      strncpy(Locale,cp,sizeof(Locale));
      setlocale(LC_ALL,Locale);
    }
  }

  // Set program defaults, can be overridden by state file or command line args
  Start_mode = FM;
  demod->L = 4096;      // Number of samples in buffer: FFT length = L + M - 1
  demod->M = 4096+1;    // Length of filter impulse response
  demod->samprate = ADC_samprate;
  demod->min_IF = -(ADC_samprate/2 - 16000); // Hardwired for Funcube
  demod->max_IF = +(ADC_samprate/2 - 16000); // Hardwired for Funcube
  demod->decimate = ADC_samprate / DAC_samprate;
  // Verify decimation ratio
  if((ADC_samprate % DAC_samprate) != 0)
    fprintf(stderr,"Warning: A/D rate %'u is not integer multiple of D/A rate %'u; decimation will probably fail\n",
	    ADC_samprate,DAC_samprate);
  
  const int N = demod->L + demod->M - 1;
  if((N % demod->decimate) != 0)
    fprintf(stderr,"Warning: FFT size %'u is not divisible by decimation ratio %d\n",N,demod->decimate);

  if((demod->M - 1) % demod->decimate != 0)
    fprintf(stderr,"Warning: Filter length %'u - 1 is not divisible by decimation ratio %d\n",demod->M,demod->decimate);

  // Must do this before first filter is created with set_mode(), otherwise a segfault can occur
  fftwf_import_system_wisdom();

  set_second_LO(demod,-ADC_samprate/4);

  // Load state file, if it exists
  // This overwrites hardwired defaults, but can be overridden with command line args
  {
    char statefile[PATH_MAX];
    snprintf(statefile,sizeof(statefile),"%s/.radiostate",getenv("HOME"));
    loadstate(demod,statefile);
  }

  // Read command line args. These supersede state file parameters, which in turn supersede program defaults
  int c;
  while((c = getopt(argc,argv,"B:c:f:i:I:k:l:L:m:M:p:R:qr:t:v")) != EOF){
    int i;

    switch(c){
    case 'B':
      OPUS_blocktime = strtod(optarg,NULL);
      break;
    case 'c':
      set_cal(demod,1e-6*strtod(optarg,NULL));
      break;
    case 'f':
      Startup_freq = parse_frequency(optarg);
      break;
    case 'i':
      set_second_LO(demod,-strtod(optarg,NULL));
      break;
    case 'I':
      strncpy(IQ_mcast_address_text,optarg,sizeof(IQ_mcast_address_text));
      break;
    case 'k':
      Kaiser_beta = strtod(optarg,NULL);
      break;
    case 'l':
      strncpy(Locale,optarg,sizeof(Locale));
      setlocale(LC_ALL,Locale);
      break;
    case 'L':
      demod->L = strtol(optarg,NULL,0);
      break;
    case 'm':
      for(i = 0; i < Nmodes;i++){
	if(strcasecmp(optarg,Modes[i].name) == 0){
	  Start_mode = Modes[i].mode;
	  break;
	}
      }
      break;
    case 'M':
      demod->M = strtol(optarg,NULL,0);
      break;
    case 'p':
      Ctl_port = strtol(optarg,NULL,0);
      break;
    case 'q':
      Quiet++; // Suppress display
      break;
    case 'r':
      OPUS_bitrate = strtol(optarg,NULL,0);
      break;
    case 'R':
      strncpy(BB_mcast_address_text,optarg,sizeof(BB_mcast_address_text));
      break;
    case 't':
      Nthreads = strtol(optarg,NULL,0);
      fftwf_init_threads();
      fftwf_plan_with_nthreads(Nthreads);
      fprintf(stderr,"Using %d threads for FFTs\n",Nthreads);
      break;
    case 'v':
      Verbose++;
      break;
    default:
      fprintf(stderr,"Usage: %s [-B opus_blocktime] [-c calibrate_ppm] [-f frequency] [-I iq multicast address] [-l locale] [-L samplepoints] [-m mode] [-M impulsepoints] [-R Audio multicast address] [-r opus_bitrate] [-t threads]\n",argv[0]);
      fprintf(stderr,"Default: %s -B %.0f -c %.2lf -f %.1f -I %s -l %s -L %d -m %s -M %d -R %s -r %d -t %d\n",
	      argv[0],OPUS_blocktime,demod->calibrate*1e6,Startup_freq,IQ_mcast_address_text,Locale,demod->L,Modes[Start_mode].name,demod->M,
	      BB_mcast_address_text,OPUS_bitrate,Nthreads);
      exit(1);
      break;
    }
  }
  if(Verbose){
    fprintf(stderr,"General coverage receiver for the Funcube Pro and Pro+\n");
    fprintf(stderr,"Copyright 2016 by Phil Karn, KA9Q; may be used under the terms of the GNU General Public License\n");
    fprintf(stderr,"Compiled %s on %s\n",__TIME__,__DATE__);
    fprintf(stderr,"Nmodes = %d\n",Nmodes);
    fprintf(stderr,"UDP control port %d\n",Ctl_port);
    fprintf(stderr,"A/D sample rate %'d, D/A sample rate %'d, decimation ratio %'d\n",
	  ADC_samprate,DAC_samprate,demod->decimate);
    fprintf(stderr,"block size: %'d complex samples (%'.1f ms @ %'u S/s)\n",
	  demod->L,1000.*demod->L/ADC_samprate,ADC_samprate);
    fprintf(stderr,"Kaiser beta %'.1lf, impulse response: %'d complex samples (%'.1f ms @ %'u S/s) bin size %'.1f Hz\n",
	    Kaiser_beta,demod->M,1000.*demod->M/ADC_samprate,ADC_samprate,(float)ADC_samprate/N);
  }

  Input_fd = setup_mcast(IQ_mcast_address_text,Mcast_dest_port,0);
  if(Input_fd == -1){
    fprintf(stderr,"Can't set up I/Q input\n");
    exit(1);
  }

  {
    // Set up control port
    // This has been neglected for a while
    struct sockaddr_in sock;
    sock.sin_family = AF_INET;
    sock.sin_port = htons(Ctl_port);
    sock.sin_addr.s_addr = INADDR_ANY;

    if((Ctl_fd = socket(PF_INET,SOCK_DGRAM, 0)) == -1)
      perror("can't open control socket");

    if(bind(Ctl_fd,(struct sockaddr *)&sock,sizeof(struct sockaddr_in)) != 0)
      perror("control bind failed");
  }
  // Set up audio output stream(s)
  Mcast_fd = setup_mcast(BB_mcast_address_text,Mcast_dest_port,1);
  if(Mcast_fd == -1){
    fprintf(stderr,"Can't set up multicast audio output\n");
    exit(1);
  }

  // Pipes for front end -> demod
  int sv[2];
  if(pipe(sv) == -1){
    perror("pipe");
    exit(1);
  }
  Demod_sock = sv[1]; // write end
  demod->input = sv[0]; // read end
#ifdef F_SETPIPE_SZ // Linux only
  int sz = fcntl(Demod_sock,F_SETPIPE_SZ,demod->L * sizeof(complex float));
#if 0
  fprintf(stderr,"sock size %d\n",sz);
#endif
  if(sz == -1)
    perror("F_SETPIPE_SZ");
#endif // F_SETPIPE_SZ
  if(setup_audio() != 0){
    fprintf(stderr,"Audio setup failed\n");
    exit(1);
  }
  if(!Quiet)
    pthread_create(&Display_thread,NULL,display,demod);

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  signal(SIGPIPE,SIG_IGN);

  assert(demod->input > 0);
  set_mode(demod,Start_mode);

  input_loop(demod); // Doesn't return

  exit(0);
}

void display_cleanup(void *);

void closedown(int a){
  if(!Quiet)
    fprintf(stderr,"radio: caught signal %d: %s\n",a,strsignal(a));
  display_cleanup(NULL);

  exit(1);
}

// Read from RTP network socket, assemble blocks of samples with corrections done

void *input_loop(struct demod *demod){
  int16_t samples[MAXPKT];
  struct rtp_header rtp;
  struct status status;
  struct iovec iovec[3];

  iovec[0].iov_base = &rtp;
  iovec[0].iov_len = sizeof(rtp);
  iovec[1].iov_base = &status;
  iovec[1].iov_len = sizeof(status);
  iovec[2].iov_base = samples;
  iovec[2].iov_len = sizeof(samples);
  
  struct msghdr message;
  message.msg_name = &Input_source_address;
  message.msg_namelen = sizeof(Input_source_address);
  message.msg_iov = iovec;
  message.msg_iovlen = sizeof(iovec) / sizeof(struct iovec);
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  int eseq = -1;

  while(1){

    assert(demod->input > 0);

    fd_set mask;
    FD_ZERO(&mask);
    FD_SET(Ctl_fd,&mask);
    FD_SET(Input_fd,&mask);

    fd_set errmask;
    FD_ZERO(&errmask);
    FD_SET(Ctl_fd,&errmask);
    FD_SET(Input_fd,&errmask);
    
    // The timeout and/or errmask recovers us if Input_fd changes, e.g., from the interactive 'I' command in display.c
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    select(max(Ctl_fd,Input_fd)+1,&mask,NULL,&errmask,&timeout);
    if(FD_ISSET(Input_fd,&errmask) || FD_ISSET(Ctl_fd,&errmask)){
      usleep(1000); // just in case we tightly loop
      continue;
    }
    if(FD_ISSET(Ctl_fd,&mask)){
      // Got a command
      socklen_t addrlen;
      addrlen = sizeof(Ctl_address);
      char pktbuf[MAXPKT];
      int rdlen;
      rdlen = recvfrom(Ctl_fd,&pktbuf,sizeof(pktbuf),0,(struct sockaddr *)&Ctl_address,&addrlen);
      // Should probably look at the source address
      if(rdlen > 0)
	process_command(demod,pktbuf,rdlen);
    }
    if(FD_ISSET(Input_fd,&mask)){
      // Receive I/Q data from front end
      int cnt;
      cnt = recvmsg(Input_fd,&message,0);
      if(cnt <= 0){    // ??
	perror("recvfrom");
	usleep(50000);
	continue;
      }
      if(cnt < sizeof(rtp) + sizeof(status))
	continue; // Too small, ignore
      
      if(Startup_freq != 0){
	// We now know where to send the tuning command for the startup frequency
	set_freq(demod,Startup_freq,1);
	Startup_freq = 0;
      }

      // Host byte order
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
      
      demod->first_LO = status.frequency;
      demod->lna_gain = status.lna_gain;
      demod->mixer_gain = status.mixer_gain;
      demod->if_gain = status.if_gain;    
      cnt -= sizeof(rtp) + sizeof(status);
      cnt /= 4; // count 4-byte stereo samples
      proc_samples(demod,samples,cnt);
    }
  }
}
int process_command(struct demod *demod,char const *cmdbuf,int len){
  struct command command;

  if(len >= sizeof(command)){
    memcpy(&command,cmdbuf,sizeof(command));
    switch(command.cmd){
    case SENDSTAT:
      break; // do this later
    case SETSTATE: // first LO freq, second LO freq, second_LO freq rate, mode
#if 0
      fprintf(stderr,"setstate(%d,%.2lf,%.2lf,%.2lf,%.2lf)\n",
	      command.mode,
	      command.first_LO,
	      command.second_LO,command.second_LO_rate,command.calibrate);
#endif
      // Ignore out-of-range values
      if(command.first_LO > 0 && command.first_LO < 2e9)
	set_first_LO(demod,command.first_LO,0);

      if(command.second_LO >= -demod->samprate/2 && command.second_LO <= demod->samprate/2)
	set_second_LO(demod,command.second_LO);
      if(fabs(command.second_LO_rate) < 1e9)
	set_second_LO_rate(demod,command.second_LO_rate,0);
      if(command.mode < Nmodes)
	set_mode(demod,command.mode);
      if(fabs(command.calibrate) < 1)
	set_cal(demod,command.calibrate);
      break;
    default:
      break; // Ignore
    } // switch
  }
  return 0;
}
 
// Save receiver state in file
int savestate(struct demod *demod,char const *statefile){
    // Dump receiver state to file
    FILE * const fp = fopen(statefile,"w");
    if(fp == NULL){
      fprintf(stderr,"Can't write state file %s\n",statefile);
    } else {
      fprintf(fp,"#KA9Q DSP Receiver State dump\n");
      fprintf(fp,"Locale %s\n",Locale);
      fprintf(fp,"Source %s %s\n",IQ_mcast_address_text,Mcast_dest_port);
      fprintf(fp,"Frequency %.3f Hz\n",get_freq(demod));
      fprintf(fp,"Mode %s\n",Modes[demod->mode].name);
      fprintf(fp,"Dial offset %.3f Hz\n",demod->dial_offset);
      fprintf(fp,"Calibrate %.3f ppm\n",get_cal(demod)*1e6);
      fprintf(fp,"Filter low %.3f Hz\n",demod->low);
      fprintf(fp,"Filter high %.3f Hz\n",demod->high);
      fprintf(fp,"Kaiser Beta %.3f\n",Kaiser_beta);
      fprintf(fp,"Blocksize %d\n",demod->L);
      fprintf(fp,"Impulse len %d\n",demod->M);
      fprintf(fp,"Tunestep %d\n",Tunestep);
      fprintf(fp,"Audio multicast address %s\n",BB_mcast_address_text);
      fprintf(fp,"Opus blocktime %.0f\n",OPUS_blocktime);
      fprintf(fp,"OPUS bitrate %d\n",OPUS_bitrate);
      fprintf(fp,"Control port %d\n",Ctl_port);
      fclose(fp);
    }
    return 0;
}
// Load receiver state from file
int loadstate(struct demod *demod,char const *statefile){
    FILE * const fp = fopen(statefile,"r");
    if(fp != NULL){
      char line[PATH_MAX];
      while(fgets(line,sizeof(line),fp) != NULL){
	chomp(line);
	double f;
	if(sscanf(line,"Frequency %lf",&Startup_freq) > 0){
	} else if(strncmp(line,"Mode ",5) == 0){
	  int i;
	  for(i=0;i<Nmodes;i++){
	    if(strncasecmp(Modes[i].name,&line[5],strlen(Modes[i].name)) == 0){
	      Start_mode = Modes[i].mode;
	      break;
	    }
	  }
	} else if(sscanf(line,"Dial offset %lf",&demod->dial_offset) > 0){
	} else if(sscanf(line,"Calibrate %lf",&f) > 0){
	  set_cal(demod,f*1e-6);
	} else if(sscanf(line,"Filter low %f",&demod->low) > 0){
	} else if(sscanf(line,"Filter high %f",&demod->high) > 0){
	} else if(sscanf(line,"Kaiser Beta %f",&Kaiser_beta) > 0){
	} else if(sscanf(line,"Blocksize %d",&demod->L) > 0){
	} else if(sscanf(line,"Impulse len %d",&demod->M) > 0){
	} else if(sscanf(line,"Tunestep %d",&Tunestep) > 0){
	} else if(sscanf(line,"Source %256s %256s",IQ_mcast_address_text,Mcast_dest_port) > 0){
	  // Sizes defined elsewhere!
	} else if(sscanf(line,"Audio multicast address %256s",BB_mcast_address_text) > 0){
	} else if(sscanf(line,"Opus blocktime %f",&OPUS_blocktime) > 0){
	} else if(sscanf(line,"OPUS bitrate %d",&OPUS_bitrate) > 0){
	} else if(sscanf(line,"Control port %d",&Ctl_port) > 0){
	} else if(sscanf(line,"Locale %256s",Locale)){
	  setlocale(LC_ALL,Locale);
	}
      }
      fclose(fp);
    }
    return 0;
}
