// $Id: main.c,v 1.125 2018/12/03 11:43:30 karn Exp karn $
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

#include "misc.h"
#include "dsp.h"
#include "multicast.h"
#include "radio.h"
#include "filter.h"
#include "status.h"


// Config constants
#define MAXPKT 1500 // Maximum bytes of data in incoming I/Q packet
char Libdir[] = "/usr/local/share/ka9q-radio";
int static DAC_samprate = 48000;

// Command line Parameters with default values
int Nthreads = 1;
int Quiet = 0;
int Verbose = 0;
char Statepath[PATH_MAX];
char Locale[256] = "en_US.UTF-8";
int Update_interval = 100;  // 100 ms between screen updates
int Mcast_ttl = 1;

// Primary control blocks for downconvert/filter/demodulate and output
// Note: initialized to all zeroes, like all global variables
struct demod Demod;

struct timeval Starttime;      // System clock at timestamp 0, for RTCP

uint64_t Commands;

void output_cleanup(void *);
void closedown(int);
void *rtp_recv(void *);
void *rtcp_send(void *);
void cleanup(void);
void closedown(int);
void decode_status(struct demod *demod,unsigned char *buffer,int length){
  unsigned char *cp = buffer;
  double nfreq = NAN;
  int gainchange = 0;

  while(cp - buffer < length){
    enum status_type type = *cp++; // increment cp to length field

    if(type == EOL)
      break; // End of list

    unsigned int len = *cp++;
    //    fprintf(stderr,"type %d len %d\n",type,len);
    if(cp - buffer + len >= length)
      break; // Invalid length
    switch(type){
    case EOL: // Shouldn't get here since it's checked above
      goto done;
    case RADIO_FREQUENCY:
      nfreq = decode_double(cp,len);
      break;
    case OUTPUT_SAMPRATE:
      demod->input.samprate = demod->status.samprate = decode_int(cp,len);
      demod->filter.decimate = demod->status.samprate / demod->output.samprate;
      break;
    case GPS_TIME:
      demod->status.timestamp = decode_int(cp,len);
      fprintf(stderr,"GPS time %lld\n",demod->status.timestamp);
      break;
    case LOW_EDGE:
      demod->min_IF = decode_float(cp,len);
      break;
    case HIGH_EDGE:
      demod->max_IF = decode_float(cp,len);
      break;
    case LNA_GAIN:
      demod->status.lna_gain = decode_int(cp,len);
      gainchange++;
      break;
    case MIXER_GAIN:
      demod->status.mixer_gain = decode_int(cp,len);
      gainchange++;
      break;
    case IF_GAIN:
      demod->status.if_gain = decode_int(cp,len);
      gainchange++;
      break;
    case DC_I_OFFSET:
      demod->DC_i = decode_float(cp,len);
      break;
    case DC_Q_OFFSET:
      demod->DC_q = decode_float(cp,len);
      break;
    case IQ_IMBALANCE:
      demod->imbalance = decode_float(cp,len);
      break;
    case IQ_PHASE:
      demod->sinphi = decode_float(cp,len);
      break;
    default:
      break;
    }
    cp += len;
  }
  if(gainchange)
    demod->gain_factor = powf(10.,-0.05*(demod->status.lna_gain + demod->status.if_gain + demod->status.mixer_gain));
  if(!isnan(nfreq) && demod->status.frequency != nfreq && demod->status.samprate != 0){
    // Recalculate LO2
    demod->status.frequency = nfreq;
    double new_LO2 = -(demod->freq - get_first_LO(demod));
    set_second_LO(demod,new_LO2);
  }
  done:;
}



void *new_fe_status(void *arg){
  struct demod *demod = (struct demod *)arg;

  while(1){
    unsigned char buffer[8192];

    memset(buffer,0,sizeof(buffer));
    int n = recv(demod->input.nctlrx_fd,buffer,sizeof(buffer),0);
    if(n <= 0){
      sleep(1);
      continue;
    }
    // Parse entries
    int cr = buffer[0]; // command-response byte
    //    fprintf(stderr,"new_fe_status len = %d, cr = %d\n",n,cr);

    if(cr == 1)
      continue; // Ignore commands
    
    decode_status(demod,buffer+1,n-1);
    pthread_mutex_lock(&demod->status_mutex);
    pthread_cond_broadcast(&demod->status_cond);
    pthread_mutex_unlock(&demod->status_mutex);
  }    
}


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
  fftwf_make_planner_thread_safe();

  struct demod * const demod = &Demod; // Only one demodulator per program for now

  // Set program defaults, can be overridden by state file and command line args, in that order
  memset(demod,0,sizeof(*demod)); // Just in case it's ever dynamic
  demod->output.samprate = DAC_samprate; // currently 48 kHz, hard to change
  strcpy(demod->mode,"FM");
  demod->freq = 147.435e6;  // LA "animal house" repeater, active all night for testing

  demod->filter.L = 3840;      // Number of samples in buffer: FFT length = L + M - 1
  demod->filter.M = 4352+1;    // Length of filter impulse response
  demod->filter.kaiser_beta = 3.0; // Reasonable compromise
  strlcpy(demod->input.dest_address_text,"iq.hf.mcast.local",sizeof(demod->input.dest_address_text));
  demod->agc.headroom = pow(10.,-15./20); // -15 dB
  strlcpy(demod->output.dest_address_text,"pcm.hf.mcast.local",sizeof(demod->output.dest_address_text));
  demod->tunestep = 0;  // single digit hertz position
  demod->imbalance = 1; // 0 dB
  demod->filter.decimate = 1; // default to avoid division by zero
  demod->filter.interpolate = 1;

  // set invalid to start
  demod->input.source_address.ss_family = -1; // Set invalid
  demod->filter.low = NAN;
  demod->filter.high = NAN;
  set_shift(demod,0);

  // Find any file argument and load it
  char optstring[] = "d:f:I:k:l:L:m:M:r:R:qs:t:T:u:vS:";
  while(getopt(argc,argv,optstring) != -1)
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
    case 'd':
      demod->doppler_command = optarg;
      break;
    case 'f':   // Initial RF tuning frequency
      demod->freq = parse_frequency(optarg);
      break;
    case 'I':   // Multicast address to listen to for I/Q data
      strlcpy(demod->input.dest_address_text,optarg,sizeof(demod->input.dest_address_text));
      break;
    case 'k':   // Kaiser window shape parameter; 0 = rectangular
      demod->filter.kaiser_beta = strtod(optarg,NULL);
      break;
    case 'l':   // Locale, mainly for numerical output format
      strlcpy(Locale,optarg,sizeof(Locale));
      setlocale(LC_ALL,Locale);
      break;
    case 'L':   // Pre-detection filter block size
      demod->filter.L = strtol(optarg,NULL,0);
      break;
    case 'm':   // receiver mode (AM/FM, etc)
      strlcpy(demod->mode,optarg,sizeof(demod->mode));
      break;
    case 'M':   // Pre-detection filter impulse length
      demod->filter.M = strtol(optarg,NULL,0);
      break;
    case 'q':
      Quiet++;  // Suppress display
      break;
    case 'R':   // Set output target IP multicast address
      strlcpy(demod->output.dest_address_text,optarg,sizeof(demod->output.dest_address_text));
      break;
    case 's':
      {
	double shift = strtod(optarg,NULL);
	set_shift(demod,shift);
      }
      break;
    case 'T': // TTL on output packets
      Mcast_ttl = strtol(optarg,NULL,0);
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
    case 'S':   // Set SSRC on output stream
      demod->output.rtp.ssrc = strtol(optarg,NULL,0);
      break;
    default:
      fprintf(stderr,"Usage: %s [-d doppler_command] [-f frequency] [-I iq multicast address] [-k kaiser_beta] [-l locale] [-L blocksize] [-m mode] [-M FIRlength] [-q] [-R Output multicast address] [-s shift offset] [-t threads] [-u update_ms] [-v]\n",argv[0]);
      exit(1);
      break;
    }
  }
  fprintf(stderr,"General coverage receiver for the Funcube Pro and Pro+\n");
  fprintf(stderr,"Copyright 2017 by Phil Karn, KA9Q; may be used under the terms of the GNU General Public License\n");
  
  // Set up actual demod state
  demod->input.ctl_fd = -1;   // Invalid
  demod->input.fd = -1; // Invalid

  pthread_mutex_init(&demod->status_mutex,NULL);
  pthread_cond_init(&demod->status_cond,NULL);
  pthread_mutex_init(&demod->doppler.mutex,NULL);
  pthread_mutex_init(&demod->shift.mutex,NULL);
  pthread_mutex_init(&demod->second_LO.mutex,NULL);
  pthread_mutex_init(&demod->qmutex,NULL);
  pthread_cond_init(&demod->qcond,NULL);

  // Input socket for I/Q data from SDR
  demod->input.fd = setup_mcast(demod->input.dest_address_text,(struct sockaddr *)&demod->input.dest_address,0,0,0);
  if(demod->input.fd == -1){
    fprintf(stderr,"Can't set up I/Q input\n");
    exit(1);
  }
  // For sending commands to front end
  if((demod->input.ctl_fd = socket(PF_INET,SOCK_DGRAM, 0)) == -1)
    perror("can't open control socket");

  demod->input.nctlrx_fd = setup_mcast(demod->input.dest_address_text,NULL,0,0,2);
  demod->input.nctltx_fd = setup_mcast(demod->input.dest_address_text,NULL,1,Mcast_ttl,2);

  gettimeofday(&Starttime,NULL);

  // Blocksize really should be computed from demod->filter.L and decimate
  if(setup_output(demod,Mcast_ttl) != 0){
    fprintf(stderr,"Output setup failed\n");
    exit(1);
  }
  // Create master half of filter
  // Must be done before the demodulator starts or it will fail an assert
  // If done in proc_samples(), will be a race condition
  demod->filter.in = create_filter_input(demod->filter.L,demod->filter.M,COMPLEX);

  pthread_create(&demod->rtp_recv_thread,NULL,rtp_recv,demod);
  pthread_create(&demod->proc_samples,NULL,proc_samples,demod);

  // Optional doppler correction
  if(demod->doppler_command)
    pthread_create(&demod->doppler_thread,NULL,doppler,demod);

  pthread_t status_thread;
  pthread_create(&status_thread,NULL,status,demod);

  pthread_t rtcp_thread;
  pthread_create(&rtcp_thread,NULL,rtcp_send,demod);

  pthread_t fe_status_thread;
  pthread_create(&fe_status_thread,NULL,new_fe_status,demod);


  // Block until we get a packet from the SDR and we know the sample rate
  fprintf(stderr,"Waiting for first SDR packet to learn sample rate...\n");
  pthread_mutex_lock(&demod->status_mutex);
  while(demod->status.samprate == 0)
    pthread_cond_wait(&demod->status_cond,&demod->status_mutex);
  pthread_mutex_unlock(&demod->status_mutex);

  // Actually set the mode and frequency already specified
  set_freq(demod,demod->freq,NAN); 
  demod->agc.gain = dB2voltage(100.0); // Empirical starting value
  set_mode(demod,demod->mode,0); // Don't override with defaults from mode table 

  // Graceful signal catch
  signal(SIGPIPE,closedown);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  signal(SIGPIPE,SIG_IGN);

  // Start the display thread unless quiet; then just twiddle our thumbs
  pthread_t display_thread;
  if(!Quiet)
    pthread_create(&display_thread,NULL,display,demod);

  while(1)
    usleep(1000000); // probably get rid of this

  exit(0);
}


// Thread to read from RTP network socket, remove DC offsets,
// fix I/Q gain and phase imbalance,
// Write corrected data to circular buffer, wake up demodulator thread(s)
// when data is available and when SDR status (frequency, sampling rate) changes
void *rtp_recv(void *arg){
  pthread_setname("rtp-rcv");
  assert(arg != NULL);
  struct demod * const demod = arg;
  
  struct packet *pkt = NULL;

  while(1){
    // Packet consists of Ethernet, IP and UDP header (already stripped)
    // then standard Real Time Protocol (RTP), a status header and the PCM
    // I/Q data. RTP is an IETF standard, so it uses big endian numbers
    // The status header and I/Q data are *not* standard, so we save time
    // by using machine byte order (almost certainly little endian).
    // Note this is a portability problem if this system and the one generating
    // the data have opposite byte orders. But who's big endian anymore?
    // Receive I/Q data from front end
    // Incoming RTP packets

    if(!pkt)
      pkt = malloc(sizeof(*pkt));

    socklen_t socksize = sizeof(demod->input.source_address);
    int size = recvfrom(demod->input.fd,pkt->content,sizeof(pkt->content),0,(struct sockaddr *)&demod->input.source_address,&socksize);
    if(size <= 0){    // ??
      perror("recvfrom");
      usleep(50000);
      continue;
    }
    if(size < RTP_MIN_SIZE)
      continue; // Too small for RTP, ignore

    unsigned char *dp = pkt->content;
    dp = ntoh_rtp(&pkt->rtp,dp);
    size -= (dp - pkt->content);
    
    if(pkt->rtp.pad){
      // Remove padding
      size -= dp[size-1];
      pkt->rtp.pad = 0;
    }
    if(pkt->rtp.type != IQ_PT && pkt->rtp.type != IQ_PT8)
      continue; // Wrong type
  
    // Unlike 'opus', 'packet', 'monitor', etc, 'radio' is nominally an interactive program so we don't keep track of SSRCs.
    // All digital IF traffic to this multicast group with the correct payload type will be accepted so that we don't have to restart
    // the single copy of 'radio' when the SDR hardware daemon restarts.
    // Maybe this should change, but it's hard to know what to do except when running as a non-interactive
    // background daemon. In that case, a new SSRC in the digital IF stream could fork a new instance of 'radio' with the same parameters,
    // and it in turn would send demodulated PCM to a new SSRC.

    // These are in host byte order, i.e., *little* endian because we don't have to interoperate with anything else
    struct status new_status;
    new_status.timestamp = *(long long *)dp;
    new_status.frequency = *(double *)&dp[8];
    new_status.samprate = *(uint32_t *)&dp[16];
    new_status.lna_gain = dp[20];
    new_status.mixer_gain = dp[21];
    new_status.if_gain = dp[22];
    dp += 24;
    size -= 24;
    update_status(demod,&new_status);

    pkt->data = dp;
    pkt->len = size;

    // Insert onto queue sorted by sequence number, wake up thread
    struct packet *q_prev = NULL;
    struct packet *qe = NULL;
    pthread_mutex_lock(&demod->qmutex);
    for(qe = demod->queue; qe && pkt->rtp.seq >= qe->rtp.seq; q_prev = qe,qe = qe->next)
      ;

    pkt->next = qe;
    if(q_prev)
      q_prev->next = pkt;
    else
      demod->queue = pkt; // Front of list

    pkt = NULL;        // force new packet to be allocated
    // wake up decoder thread
    pthread_cond_signal(&demod->qcond);
    pthread_mutex_unlock(&demod->qmutex);
  }      
  return NULL;
}


// Save receiver state to file
// Path is Statepath[] = $HOME/.radiostate
int savestate(struct demod *dp,char const *filename){
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
  fprintf(fp,"Source %s\n",dp->input.dest_address_text);
  fprintf(fp,"Output %s\n",dp->output.dest_address_text);
  fprintf(fp,"TTL %d\n",Mcast_ttl);
  fprintf(fp,"Blocksize %d\n",dp->filter.L);
  fprintf(fp,"Impulse len %d\n",dp->filter.M);
  fprintf(fp,"Frequency %.3f Hz\n",get_freq(dp));
  fprintf(fp,"Mode %s\n",dp->mode);
  fprintf(fp,"Shift %.3f Hz\n",dp->shift.freq);
  fprintf(fp,"Filter low %.3f Hz\n",dp->filter.low);
  fprintf(fp,"Filter high %.3f Hz\n",dp->filter.high);
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
    if(sscanf(line,"Frequency %lf",&dp->freq) > 0){
    } else if(strncmp(line,"Mode ",5) == 0){
      strlcpy(dp->mode,&line[5],sizeof(dp->mode));
    } else if(sscanf(line,"Shift %lf",&dp->shift.freq) > 0){
    } else if(sscanf(line,"Filter low %f",&dp->filter.low) > 0){
    } else if(sscanf(line,"Filter high %f",&dp->filter.high) > 0){
    } else if(sscanf(line,"Kaiser Beta %f",&dp->filter.kaiser_beta) > 0){
    } else if(sscanf(line,"Blocksize %d",&dp->filter.L) > 0){
    } else if(sscanf(line,"Impulse len %d",&dp->filter.M) > 0){
    } else if(sscanf(line,"Tunestep %d",&dp->tunestep) > 0){
    } else if(sscanf(line,"Source %256s",dp->input.dest_address_text) > 0){
      // Array sizes defined elsewhere!
    } else if(sscanf(line,"Output %256s",dp->output.dest_address_text) > 0){
    } else if(sscanf(line,"TTL %d",&Mcast_ttl) > 0){
    } else if(sscanf(line,"Locale %256s",Locale)){
      setlocale(LC_ALL,Locale);
    }
  }
  fclose(fp);
  return 0;
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
  if(!Quiet)
    fprintf(stderr,"Signal %d\n",a);
  exit(1);
}
