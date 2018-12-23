// $Id: main.c,v 1.143 2018/12/22 10:12:04 karn Exp karn $
// Read complex float samples from multicast stream (e.g., from funcube.c)
// downconvert, filter, demodulate, optionally compress and multicast output
// Copyright 2017, Phil Karn, KA9Q, karn@ka9q.net
#define _GNU_SOURCE 1
#include <assert.h>
#include <errno.h>
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
#include <locale.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <signal.h>
#include <getopt.h>

#include "misc.h"
#include "dsp.h"
#include "multicast.h"
#include "radio.h"
#include "filter.h"
#include "status.h"


// Config constants
char Libdir[] = "/usr/local/share/ka9q-radio";
int DAC_samprate = 48000;

// Command line Parameters with default values
int Nthreads = 1;
char const *Locale = "en_US.UTF-8";
int Mcast_ttl = 1;
int Blocktime = 20; // 20 milliseconds

// Primary control blocks for downconvert/filter/demodulate and output
// Note: initialized to all zeroes, like all global variables
struct demod Demod;

struct timeval Starttime;      // System clock at timestamp 0, for RTCP

extern uint64_t Commands;

void output_cleanup(void *);
void closedown(int);
void *rtp_recv(void *);
void *rtcp_send(void *);
void cleanup(void);
void closedown(int);
void *send_status(void *);

struct option Options[] =
  {
   {"pcm-out", required_argument, NULL, 'D'},
   {"flat", no_argument, NULL, 'F'},
   {"agc-hangtime", required_argument, NULL, 'H'},
   {"status-in", required_argument, NULL, 'I'},
   {"status-out", required_argument, NULL, 'R'},
   {"ssrc", required_argument, NULL, 'S'},
   {"ttl", required_argument, NULL, 'T'},
   {"agc-recover", required_argument, NULL, 'a'},
   {"block-time", required_argument, NULL, 'b'},
   {"channels", required_argument, NULL, 'c'},
   {"env", no_argument, NULL, 'e'},
   {"frequency", required_argument, NULL, 'f'},
   {"filter-high", required_argument, NULL, 'h'},
   {"isb", no_argument, NULL, 'i'},
   {"kaiser-beta", required_argument, NULL, 'k'},
   {"filter-low", required_argument, NULL, 'l'},
   {"mode", required_argument, NULL, 'm'},
   {"pll", no_argument, NULL, 'p'},
   {"square", no_argument, NULL, 'q'},
   {"headroom", required_argument, NULL, 'r'},
   {"shift", required_argument, NULL, 's'},
   {"fft-threads", required_argument, NULL, 't'},
   {NULL, 0, NULL, 0},
  };

char Optstring[] = "D:FI:R:S:T:a:b:c:e:f:h:ik:l:m:pqr:s:t:";


// The main program sets up the demodulator parameter defaults,
// overwrites them with command-line arguments and/or state file settings,
// initializes the various local oscillators, pthread mutexes and conditions
// sets up multicast I/Q input and PCM audio output
// Sets up the input half of the pre-detection filter
// starts the RTP input and downconverter/filter threads
// sets the initial demodulation mode, which starts the demodulator thread
// catches signals and eventually becomes the user interface/display loop
int main(int argc,char *argv[]){
  // if we have root, up our priority and drop privileges
  int prio = getpriority(PRIO_PROCESS,0);
  prio = setpriority(PRIO_PROCESS,0,prio - 10);

  // Quickly drop root if we have it
  // The sooner we do this, the fewer options there are for abuse
  if(seteuid(getuid()) != 0)
    perror("seteuid");

  gettimeofday(&Starttime,NULL);

  // Set up program defaults
  // Some can be overridden by command line args
  {
    // The display thread assumes en_US.UTF-8, or anything with a thousands grouping character
    // Otherwise the cursor movements will be wrong
    char const * const cp = getenv("LANG");
    if(cp != NULL)
      Locale = cp;
  }
  setlocale(LC_ALL,Locale); // Set either the hardwired default or the value of $LANG if it exists
  fprintf(stderr,"General coverage software receiver\n");
  fprintf(stderr,"Copyright 2018 by Phil Karn, KA9Q; may be used under the terms of the GNU General Public License\n");
  if(readmodes("modes.txt") != 0){
    fprintf(stderr,"Warning: can't read mode table; --modes won't work\n");
  }
  
  // Must do this before first filter is created, otherwise a segfault can occur
  fftwf_import_system_wisdom();
  fftwf_make_planner_thread_safe();

  struct demod * const demod = &Demod; // Only one demodulator per program for now
  memset(demod,0,sizeof(*demod)); // Just in case it's ever dynamic

  demod->output.samprate = DAC_samprate; // currently 48 kHz, hard to change
  demod->filter.interpolate = 1;
  // Set receiver defaults, can be overridden by command line args
  demod->filter.kaiser_beta = 3.0; // Reasonable compromise
  demod->agc.headroom = pow(10.,-15./20); // -15 dB
  demod->sdr.imbalance = 1; // 0 dB
  demod->demod_type = 1; // FM
  demod->opt.agc = 1; // AGC is on by default
  demod->agc.gain = dB2voltage(80.); // Empirical starting point
  demod->agc.recovery_rate = powf(10.,6/20./demod->output.samprate);
  demod->output.channels = 1;
  demod->tune.freq = 147.435e6;  // LA "animal house" repeater, active all night for testing
  demod->filter.high = 8000;
  demod->filter.low = -8000;
  demod->output.rtp.ssrc = Starttime.tv_sec & 0xffffffff;
  demod->output.status_fd = demod->output.ctl_fd = demod->output.data_fd = demod->output.rtcp_fd = -1;

  pthread_mutex_init(&demod->sdr.status_mutex,NULL);
  pthread_cond_init(&demod->sdr.status_cond,NULL);
  pthread_mutex_init(&demod->doppler.mutex,NULL);
  pthread_mutex_init(&demod->shift.mutex,NULL);
  pthread_mutex_init(&demod->second_LO.mutex,NULL);
  pthread_mutex_init(&demod->demod_mutex,NULL);
  pthread_cond_init(&demod->demod_cond,NULL);

  demod->input.status_fd = -1;
  
  // First pass over options to pick up I/O sockets
  // -T must be specified ahead of output argument it modifies
  int c;
  while((c = getopt_long(argc,argv,Optstring,Options,NULL)) != -1){
    switch(c){
    case 'I':   // Multicast address to listen to for receiver metadata
      // Input socket for status from SDR
      demod->input.status_fd = setup_mcast(optarg,(struct sockaddr *)&demod->input.metadata_dest_address,0,0,2);
      if(demod->input.status_fd == -1){
	fprintf(stderr,"Can't set up SDR status socket\n");
	exit(1);
      }
      break;
    case 'R':   // Set output target IP multicast address for metadata
      demod->output.status_fd = setup_mcast(optarg,(struct sockaddr *)&demod->output.metadata_dest_address,1,Mcast_ttl,2); // RTP port + 2
      if(demod->output.status_fd == -1){
	fprintf(stderr,"Can't send status to %s\n",optarg);
      } else {
	socklen_t len = sizeof(demod->output.metadata_source_address);
	getsockname(demod->output.status_fd,(struct sockaddr *)&demod->output.metadata_source_address,&len);  
	// Same remote socket as status
	demod->output.ctl_fd = setup_mcast(NULL,(struct sockaddr *)&demod->output.metadata_dest_address,0,Mcast_ttl,2);
	if(demod->output.ctl_fd == -1){
	  fprintf(stderr,"Can't listen for commands!\n");
	}
      }
      break;
    case 'D': // target multicast group for pcm data output
      demod->output.data_fd = setup_mcast(optarg,(struct sockaddr *)&demod->output.data_dest_address,1,Mcast_ttl,0);
      if(demod->output.data_fd == -1){
	fprintf(stderr,"Can't set up PCM output\n");
	exit(1);
      }
      {
	socklen_t len = sizeof(demod->output.data_source_address);
	getsockname(demod->output.data_fd,(struct sockaddr *)&demod->output.data_source_address,&len);
      }
      demod->output.rtcp_fd = setup_mcast(optarg,NULL,1,Mcast_ttl,1); // RTP port number + 1
      if(demod->output.rtcp_fd == -1)
	fprintf(stderr,"Can't set up RTCP output\n");
      break;
    case 'S':   // Set SSRC on output stream
      demod->output.rtp.ssrc = strtol(optarg,NULL,0);
      break;
    case 'T': // TTL on output packets
      Mcast_ttl = strtol(optarg,NULL,0);
      break;
    default: // Ignore others for now
      break;
    }
  }
  if(demod->input.status_fd == -1){
    fprintf(stderr,"No valid SDR metadata address given with -I\n");
    exit(1);
  }

  // Output socket for commands to SDR - same group as metadata input
  demod->input.ctl_fd = setup_mcast(NULL,(struct sockaddr *)&demod->input.metadata_dest_address,1,Mcast_ttl,0);
  if(demod->input.ctl_fd == -1){
    fprintf(stderr,"Can't set up SDR control socket\n");
    exit(1);
  }
  // Start status thread - will also listen for SDR commands
  pthread_t status_thread;
  pthread_create(&status_thread,NULL,send_status,demod);


  fprintf(stderr,"Waiting for SDR metadata..."); fflush(stderr);
  pthread_mutex_lock(&demod->sdr.status_mutex);
  while(demod->input.samprate == 0 || demod->input.data_dest_address.ss_family == 0)
    pthread_cond_wait(&demod->sdr.status_cond,&demod->sdr.status_mutex);
  pthread_mutex_unlock(&demod->sdr.status_mutex);
  fprintf(stderr,"%'d Hz\n",demod->input.samprate);

  // Input socket for I/Q data from SDR, set from OUTPUT_DEST_SOCKET in SDR metadata
  demod->input.data_fd = setup_mcast(NULL,(struct sockaddr *)&demod->input.data_dest_address,0,0,0);
  if(demod->input.data_fd == -1){
    fprintf(stderr,"Can't set up I/Q input\n");
    exit(1);
  }

  // Go back and re-read rest of args
  optind = 1;
  while((c = getopt_long(argc,argv,Optstring,Options,NULL)) != -1){
    switch(c){
    case 'a': // AGC recovery rate, dB/s
      // Convert to ratio per sample
      demod->agc.recovery_rate = fabs(strtof(optarg,NULL)) / demod->output.samprate;
      demod->agc.recovery_rate = powf(10.,demod->agc.recovery_rate/20.);
      break;
    case 'b': // FFT convolver block duration
      Blocktime = strtol(optarg,NULL,0);
      break;
    case 'c': // Output channels
      demod->output.channels = strtol(optarg,NULL,0);
      break;
    case 'e':
      demod->opt.env = 1;
      break;
    case 'f':   // Initial RF tuning frequency
      demod->tune.freq = parse_frequency(optarg);
      break;
    case 'h':
      demod->filter.high = strtof(optarg,NULL);
      break;
    case 'i':
      demod->filter.isb = 1;
      break;
    case 'k':   // Kaiser window shape parameter; 0 = rectangular
      demod->filter.kaiser_beta = strtof(optarg,NULL);
      break;
    case 'l':
      demod->filter.low = strtof(optarg,NULL);
      break;
    case 'm':
      preset_mode(demod,optarg);
      break;
    case 'p':
      demod->opt.pll = 1;
      break;
    case 'q':
      demod->opt.square = 1;
      break;
    case 'r':
      demod->agc.headroom = -fabs(strtof(optarg,NULL));
      demod->agc.headroom = powf(10.,demod->agc.headroom/10.);
      break;
    case 's':
      demod->tune.shift = strtod(optarg,NULL);
      break;
    case 't':   // # of threads to use in FFTW3
      Nthreads = strtol(optarg,NULL,0);
      fftwf_init_threads();
      fftwf_plan_with_nthreads(Nthreads);
      fprintf(stderr,"Using %d threads for FFTs\n",Nthreads);
      break;
    case 'F':
      demod->opt.flat = 1;
      break;
    case 'H':
      demod->agc.hangtime = strtod(optarg,NULL) * demod->output.samprate;
      break;
    case 'I': case 'R': case 'D': case 'S': case 'T':
      break;
    default:
      fprintf(stderr,"option %c unknown\n",c);
      break;
    }
  }
  // Start emitting RTCP
  pthread_t rtcp_thread;
  if(demod->output.rtcp_fd != -1)
    pthread_create(&rtcp_thread,NULL,rtcp_send,demod);


  // Create filter now that we know the parameters
  // FFT and filter sizes now computed from specified block duration and sample rate
  // L = data block size
  // M = filter impulse response duration
  // N = FFT size (power of 2) = L + M - 1
  demod->filter.L = demod->input.samprate * Blocktime / 1000; // Blocktime is in milliseconds
  // FFT size N is next power of 2 larger than 2*L, so M = N - L + 1
  int N = 1 << ((int)(log2(2.0 * demod->filter.L) + 1));
  demod->filter.M = N - demod->filter.L + 1;

  demod->filter.in = create_filter_input(demod->filter.L,demod->filter.M,COMPLEX);
  demod->filter.out = create_filter_output(demod->filter.in,NULL,demod->filter.decimate,demod->filter.isb ? CROSS_CONJ : COMPLEX);
  set_filter(demod->filter.out,
	     demod->filter.low/demod->output.samprate,
	     demod->filter.high/demod->output.samprate,
	     demod->filter.kaiser_beta);

  // Start processing I/Q data stream
  pthread_t proc_samples_thread;
  pthread_create(&proc_samples_thread,NULL,proc_samples,demod);

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  signal(SIGPIPE,SIG_IGN);

  set_shift(demod,demod->tune.shift);
  set_freq(demod,demod->tune.freq,NAN);
  // Start demodulators
  pthread_create(&demod->fm_demod_thread,NULL,demod_fm,demod);  
  pthread_create(&demod->linear_demod_thread,NULL,demod_linear,demod);

  while(1){
    usleep(1000000); // probably get rid of this
  }
  exit(0);
}




// RTP control protocol sender task
void *rtcp_send(void *arg){
  struct demod *demod = (struct demod *)arg;
  if(demod == NULL)
    pthread_exit(NULL);

  pthread_setname("rtcp");
  //  fprintf(stderr,"hello from rtcp_send\n");
  while(1){

    if(demod->output.rtp.ssrc == 0) // Wait until it's set by output RTP subsystem
      goto done;
    unsigned char buffer[4096]; // much larger than necessary
    memset(buffer,0,sizeof(buffer));
    
    // Construct sender report
    struct rtcp_sr sr;
    memset(&sr,0,sizeof(sr));
    sr.ssrc = demod->output.rtp.ssrc;

    // Construct NTP timestamp
    struct timeval tv;
    gettimeofday(&tv,NULL);
    double runtime = (tv.tv_sec - Starttime.tv_sec) + (tv.tv_usec - Starttime.tv_usec)/1000000.;

    long long now_time = ((long long)tv.tv_sec + NTP_EPOCH)<< 32;
    now_time += ((long long)tv.tv_usec << 32) / 1000000;

    sr.ntp_timestamp = now_time;
    // The zero is to remind me that I start timestamps at zero, but they could start anywhere
    sr.rtp_timestamp = 0 + runtime * 48000;
    sr.packet_count = demod->output.rtp.seq;
    sr.byte_count = demod->output.rtp.bytes;
    
    unsigned char *dp = gen_sr(buffer,sizeof(buffer),&sr,NULL,0);

    // Construct SDES
    struct rtcp_sdes sdes[4];
    
    // CNAME
    char hostname[1024];
    gethostname(hostname,sizeof(hostname));
    char *string = NULL;
    int sl = asprintf(&string,"radio@%s",hostname);
    if(sl > 0 && sl <= 255){
      sdes[0].type = CNAME;
      strcpy(sdes[0].message,string);
      sdes[0].mlen = strlen(sdes[0].message);
    }
    if(string){
      free(string); string = NULL;
    }

    sdes[1].type = NAME;
    strcpy(sdes[1].message,"KA9Q Radio Program");
    sdes[1].mlen = strlen(sdes[1].message);
    
    sdes[2].type = EMAIL;
    strcpy(sdes[2].message,"karn@ka9q.net");
    sdes[2].mlen = strlen(sdes[2].message);

    sdes[3].type = TOOL;
    strcpy(sdes[3].message,"KA9Q Radio Program");
    sdes[3].mlen = strlen(sdes[3].message);
    
    dp = gen_sdes(dp,sizeof(buffer) - (dp-buffer),demod->output.rtp.ssrc,sdes,4);


    send(demod->output.rtcp_fd,buffer,dp-buffer,0);
  done:;
    usleep(1000000);
  }
}
void closedown(int a){
  fprintf(stderr,"Signal %d\n",a);
  exit(1);
}
