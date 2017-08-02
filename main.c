// $Id: main.c,v 1.54 2017/08/01 00:33:32 karn Exp karn $
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
#include <sys/stat.h>
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
struct demod Demod; // Note: initialized to all zeroes, like all global variables
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
char Mcast_dest_port[256] = "5004";
char Statepath[PATH_MAX];
char Locale[256] = "en_US.UTF-8";

int main(int argc,char *argv[]){
  {
    // The display thread assumes en_US.UTF-8, or anything with a thousands grouping character
    // Otherwise the cursor movements will be wrong
    char const * const cp = getenv("LANG");
    if(cp != NULL)
      strncpy(Locale,cp,sizeof(Locale));
  }

  // Set program defaults, can be overridden by state file or command line args
  setlocale(LC_ALL,Locale); // Set either the hardwired default or the value of $LANG if it exists

  struct demod * const demod = &Demod;
 
  // Don't let set_mode() write an eof to stdin, which it would do with the default value (0) of corr_iq_write_fd
  demod->corr_iq_write_fd = demod->corr_iq_read_fd = -1;
  demod->L = 4096;      // Number of samples in buffer: FFT length = L + M - 1
  demod->M = 4096+1;    // Length of filter impulse response
  // Must do this before first filter is created with set_mode(), otherwise a segfault can occur
  fftwf_import_system_wisdom();

  set_second_LO(demod,0); // The first set_freq will change this

  // Default state file is $HOME/.radiostate/default
  snprintf(Statepath,sizeof(Statepath),"%s/.radiostate",getenv("HOME"));
  // If this directory doesn't exist, create it
  struct stat statbuf;
  if(stat(Statepath,&statbuf) == -1){
    if(mkdir(Statepath,0755) == -1)
      fprintf(stderr,"Can't create state directory %s\n",Statepath);
  } else if((statbuf.st_mode & S_IFDIR) == 0){
    fprintf(stderr,"%s is not a directory\n",Statepath);
  } else {
    snprintf(Statepath,sizeof(Statepath),"%s/.radiostate/default",getenv("HOME"));
    loadstate(demod,Statepath); // Load default state file, if it exists
  }
  // Read command line args. Ordering is important; everything overrides the default state file,
  // but parameters given before an explicit state file will be overriden by the state file
  int c;
  while((c = getopt(argc,argv,"B:c:s:f:I:k:l:L:m:M:p:R:qr:t:v")) != EOF){
    int i;

    switch(c){
    case 'B':
      Audio.opus_blocktime = strtod(optarg,NULL);
      break;
    case 'c':
      set_cal(demod,1e-6*strtod(optarg,NULL));
      break;
    case 's':
      // Look in current directory first
      if(loadstate(demod,optarg) == 0)
	break;

      // Look in state directory
      snprintf(Statepath,sizeof(Statepath),"%s/.radiostate/%s",getenv("HOME"),optarg);
      if(loadstate(demod,Statepath) != 0)
	fprintf(stderr,"Can't load state file %s\n",Statepath);
      break;
    case 'f':
      demod->startup_freq = parse_frequency(optarg);
      break;
    case 'I':
      strncpy(demod->iq_mcast_address_text,optarg,sizeof(demod->iq_mcast_address_text));
      break;
    case 'k':
      demod->kaiser_beta = strtod(optarg,NULL);
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
	  demod->start_mode = Modes[i].mode;
	  break;
	}
      }
      break;
    case 'M':
      demod->M = strtol(optarg,NULL,0);
      break;
    case 'p':
      demod->ctl_port = strtol(optarg,NULL,0);
      break;
    case 'q':
      Quiet++; // Suppress display
      break;
    case 'r':
      Audio.opus_bitrate = strtol(optarg,NULL,0);
      break;
    case 'R':
      strncpy(Audio.audio_mcast_address_text,optarg,sizeof(Audio.audio_mcast_address_text));
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
      fprintf(stderr,"Usage: %s [-B opus_blocktime] [-c calibrate_ppm] [-d statefile] [-f frequency] [-I iq multicast address] [-l locale] [-L samplepoints] [-m mode] [-M impulsepoints] [-R Audio multicast address] [-r opus_bitrate] [-t threads] [-v]\n",argv[0]);
      fprintf(stderr,"Default: %s -B %.0f -c %.2lf -d %s -f %.1f -I %s -l %s -L %d -m %s -M %d -R %s -r %d -t %d\n",
	      argv[0],Audio.opus_blocktime,demod->calibrate*1e6,Statepath,demod->startup_freq,demod->iq_mcast_address_text,Locale,demod->L,Modes[demod->start_mode].name,demod->M,
	      Audio.audio_mcast_address_text,Audio.opus_bitrate,Nthreads);
      exit(1);
      break;
    }
  }

  if(Verbose){
    fprintf(stderr,"General coverage receiver for the Funcube Pro and Pro+\n");
    fprintf(stderr,"Copyright 2016 by Phil Karn, KA9Q; may be used under the terms of the GNU General Public License\n");
    fprintf(stderr,"Compiled %s on %s\n",__TIME__,__DATE__);
    fprintf(stderr,"Nmodes = %d\n",Nmodes);
    fprintf(stderr,"UDP control port %d\n",demod->ctl_port);
    fprintf(stderr,"D/A sample rate %'d\n",DAC_samprate);
    fprintf(stderr,"block size: %'d complex samples\n",demod->L);
    fprintf(stderr,"Kaiser beta %'.1lf, impulse response: %'d complex samples\n",demod->kaiser_beta,demod->M);
  }

  demod->input_fd = setup_mcast(demod->iq_mcast_address_text,Mcast_dest_port,0);
  if(demod->input_fd == -1){
    fprintf(stderr,"Can't set up I/Q input\n");
    exit(1);
  }

  {
    // Set up control port
    // This has been neglected for a while
    struct sockaddr_in sock;
    sock.sin_family = AF_INET;
    sock.sin_port = htons(demod->ctl_port);
    sock.sin_addr.s_addr = INADDR_ANY;

    if((demod->ctl_fd = socket(PF_INET,SOCK_DGRAM, 0)) == -1)
      perror("can't open control socket");

    if(bind(demod->ctl_fd,(struct sockaddr *)&sock,sizeof(struct sockaddr_in)) != 0)
      perror("control bind failed");
  }
  if(setup_audio(&Audio) != 0){
    fprintf(stderr,"Audio setup failed\n");
    exit(1);
  }

  demod->audio = &Audio; // Link to audio output system

  if(!Quiet)
    pthread_create(&Display_thread,NULL,display,demod);

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  signal(SIGPIPE,SIG_IGN);

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
  message.msg_name = &demod->input_source_address;
  message.msg_namelen = sizeof(demod->input_source_address);
  message.msg_iov = iovec;
  message.msg_iovlen = sizeof(iovec) / sizeof(struct iovec);
  message.msg_control = NULL;
  message.msg_controllen = 0;
  message.msg_flags = 0;

  int eseq = -1;

  while(1){

    fd_set mask;
    FD_ZERO(&mask);
    FD_SET(demod->ctl_fd,&mask);
    FD_SET(demod->input_fd,&mask);

    fd_set errmask;
    FD_ZERO(&errmask);
    FD_SET(demod->ctl_fd,&errmask);
    FD_SET(demod->input_fd,&errmask);
    
    // The timeout and/or errmask recovers us if demod->input_fd changes, e.g., from the interactive 'I' command in display.c
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    select(FD_SETSIZE,&mask,NULL,&errmask,&timeout);
    if(FD_ISSET(demod->input_fd,&errmask) || FD_ISSET(demod->ctl_fd,&errmask)){
      usleep(1000); // just in case we tightly loop
      continue;
    }
    if(FD_ISSET(demod->ctl_fd,&mask)){
      // Got a command
      socklen_t addrlen;
      addrlen = sizeof(demod->ctl_address);
      char pktbuf[MAXPKT];
      int rdlen;
      rdlen = recvfrom(demod->ctl_fd,&pktbuf,sizeof(pktbuf),0,(struct sockaddr *)&demod->ctl_address,&addrlen);
      // Should probably look at the source address
      if(rdlen > 0)
	process_command(demod,pktbuf,rdlen);
    }
    if(FD_ISSET(demod->input_fd,&mask)){
      // Receive I/Q data from front end
      int cnt;
      cnt = recvmsg(demod->input_fd,&message,0);
      if(cnt <= 0){    // ??
	perror("recvfrom");
	usleep(50000);
	continue;
      }
      if(cnt < sizeof(rtp) + sizeof(status))
	continue; // Too small, ignore
      
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

      if(status.samprate != demod->nominal_samprate){
	// We now know the A/D sample rate, or it's changed
	demod->nominal_samprate = status.samprate;
	demod->samprate = demod->nominal_samprate * (1 + demod->calibrate * 1e-6);
	demod->max_IF = demod->nominal_samprate/2 - 16000; // Hardwired for Funcube, make this more general somehow
	demod->min_IF = -demod->max_IF;
	demod->decimate = demod->nominal_samprate / DAC_samprate;
      }

      demod->first_LO = status.frequency;
      demod->lna_gain = status.lna_gain;
      demod->mixer_gain = status.mixer_gain;
      demod->if_gain = status.if_gain;    
      if(demod->startup_freq != 0){
	set_freq(demod,demod->startup_freq,1); // Happens on the very first I/Q data packet
	demod->startup_freq = 0;
      }
      if(demod->start_mode != NO_MODE){
	set_mode(demod,demod->start_mode); // Need to recompute filters - how do we keep statefile overrides?
	demod->start_mode = NO_MODE;
      }
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
    fprintf(fp,"Source %s %s\n",demod->iq_mcast_address_text,Mcast_dest_port);
    fprintf(fp,"Audio multicast address %s\n",Audio.audio_mcast_address_text);
    fprintf(fp,"Opus blocktime %.0f\n",Audio.opus_blocktime);
    fprintf(fp,"OPUS bitrate %d\n",Audio.opus_bitrate);
    fprintf(fp,"Control port %d\n",demod->ctl_port);
    fprintf(fp,"Blocksize %d\n",demod->L);
    fprintf(fp,"Impulse len %d\n",demod->M);
    fprintf(fp,"Frequency %.3f Hz\n",get_freq(demod));
    fprintf(fp,"Mode %s\n",Modes[demod->mode].name);
    fprintf(fp,"Dial offset %.3f Hz\n",demod->dial_offset);
    fprintf(fp,"Kaiser Beta %.3f\n",demod->kaiser_beta);
    fprintf(fp,"Filter low %.3f Hz\n",demod->low);
    fprintf(fp,"Filter high %.3f Hz\n",demod->high);
    fprintf(fp,"Tunestep %d\n",Tunestep);
    fprintf(fp,"Calibrate %.3f ppm\n",demod->calibrate*1e6); // do last?
    fclose(fp);
  }
  return 0;
}
// Load receiver state from file
// Some of these are problematic since they're overwritten from the mode
// table when the mode is actually set on the first A/D packet:
// dial_offset, filter low, filter high, tuning step (not currently set)
// 
int loadstate(struct demod *demod,char const *statefile){
  FILE * const fp = fopen(statefile,"r");
  if(fp == NULL)
    return -1;

  char line[PATH_MAX];

  while(fgets(line,sizeof(line),fp) != NULL){
    chomp(line);
    double f;
    if(sscanf(line,"Frequency %lf",&demod->startup_freq) > 0){
    } else if(strncmp(line,"Mode ",5) == 0){
      int i;
      for(i=0;i<Nmodes;i++){
	if(strncasecmp(Modes[i].name,&line[5],strlen(Modes[i].name)) == 0){
	  demod->start_mode = Modes[i].mode;
	  break;
	}
      }
    } else if(sscanf(line,"Dial offset %lf",&demod->dial_offset) > 0){
    } else if(sscanf(line,"Filter low %f",&demod->low) > 0){
    } else if(sscanf(line,"Filter high %f",&demod->high) > 0){
    } else if(sscanf(line,"Kaiser Beta %f",&demod->kaiser_beta) > 0){
    } else if(sscanf(line,"Blocksize %d",&demod->L) > 0){
    } else if(sscanf(line,"Impulse len %d",&demod->M) > 0){
    } else if(sscanf(line,"Tunestep %d",&Tunestep) > 0){
    } else if(sscanf(line,"Source %256s %256s",demod->iq_mcast_address_text,Mcast_dest_port) > 0){
      // Array sizes defined elsewhere!
    } else if(sscanf(line,"Audio multicast address %256s",Audio.audio_mcast_address_text) > 0){
    } else if(sscanf(line,"Opus blocktime %f",&Audio.opus_blocktime) > 0){
    } else if(sscanf(line,"OPUS bitrate %d",&Audio.opus_bitrate) > 0){
    } else if(sscanf(line,"Control port %d",&demod->ctl_port) > 0){
    } else if(sscanf(line,"Locale %256s",Locale)){
      setlocale(LC_ALL,Locale);
    } else if(sscanf(line,"Calibrate %lf",&f) > 0){
      demod->calibrate = f * 1e-6;
    }
  }
  fclose(fp);
  return 0;
}
