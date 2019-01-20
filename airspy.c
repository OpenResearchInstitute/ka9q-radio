// $Id$
// Read from Airspy SDR
// Accept control commands from UDP socket
#define _GNU_SOURCE 1
#include <assert.h>
#include <pthread.h>
#include <string.h>
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <locale.h>
#include <sys/time.h>
#include <libairspy/airspy.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>
#include <syslog.h>
#include <sys/stat.h>
#include <getopt.h>

#include "sdr.h"
#include "misc.h"
#include "multicast.h"
#include "decimate.h"
#include "status.h"
#include "dsp.h"

#define N_serials 20
uint64_t Serials[N_serials];
uint32_t Sample_rates[1000];


struct sdrstate {
  struct airspy_device *device;    // Opaque pointer
  struct status status;     // Frequency and gain settings, grouped for transmission in RTP packet
  uint64_t intfreq;
  float in_power;           // Running estimate of unfiltered A/D signal power
  float out_power;          // Filtered output power

  // Smoothed error estimates
  complex float DC;      // DC offset
  float sinphi;          // I/Q phase error
  float imbalance;       // Ratio of I power to Q power
  double calibration;

  int clips;                // Sample clips since last reset
  uint32_t command_tag;
};


// Configurable parameters
// decibel limits for power
float AGC_upper = -15;
float AGC_lower = -25;
int ADC_samprate; // Computed from Out_samprate * Decimate
float Rate_factor; // Computed from ADC_samprate and Power_alpha
int Out_samprate = 192000;
int Decimate = 64;
int Log_decimate = 6; // Computed from Decimate
const float SCALE8 = 1./127.;   // Scale 8-bit samples to unity range floats
const int Bufsize = 16384;
float const DC_alpha = .001;  // high pass filter coefficient for DC offset estimates, per callback block
float const Power_alpha= 1.0; // time constant (seconds) for smoothing power and I/Q imbalance estimates
char *Rundir = "/run/airspy"; // Where 'status' and 'pid' get written
float Filter_atten;
int const stage_threshold = 8; // point at which to switch to filter f8
#define BUFFERSIZE  (1<<19) // Upcalls seem to be 256KB; don't make too big or we may blow out of the cache

// Variables set by command line options
int Blocksize = 350; // Safe for 16-bit samples at 1500 byte MTU
int Device = 0;      // Which of several to use
char *Locale;
int Offset=1;     // Default to offset high by +Fs/4 downconvert in software to avoid DC
int Daemonize = 0;
int Mcast_ttl = 1; // Don't send fast IQ streams beyond the local network by default
char *Data_dest = "239.1.6.10";
char *Metadata_dest = "239.1.6.1"; // Default for testing
double Frequency = 146e6;
int Rtp_type = IQ_PT; // Default to old 16-bit little-endian with metadata
int Verbose;

struct sockaddr_storage Output_metadata_dest_address;

struct option Options[] =
  {
   {"iface", required_argument, NULL, 'A'},
   {"pcm-out", required_argument, NULL, 'D'},
   {"iq-out", required_argument, NULL, 'D'},
   {"device", required_argument, NULL, 'I'},
   {"status-out", required_argument, NULL, 'R'},
   {"ssrc", required_argument, NULL, 'S'},
   {"ttl", required_argument, NULL, 'T'},
   {"blocksize", required_argument, NULL, 'b'},
   {"decimate", required_argument, NULL, 'c'},
   {"daemonize", no_argument, NULL, 'd'},
   {"frequency", required_argument, NULL, 'f'},
   {"offset", required_argument, NULL, 'o'},
   {"samprate", required_argument, NULL, 'r'},
   {"sample-rate", required_argument, NULL, 'r'},
   {"rtp-type", required_argument, NULL, 't'},
   {"verbose", no_argument, NULL, 'v'},
   {NULL, 0, NULL, 0},
  };
char Optstring[] = "A:D:I:R:S:T:b:c:df:o:r:t:v";


// Global variables
struct rtp_state Rtp;
int Rtp_sock;     // Socket handle for sending real time stream
int Nctl_sock;    // Socket handle for incoming commands
int Status_sock;  // Socket handle for outgoing status messages
struct sockaddr_storage Output_data_dest_address; // Multicast output socket
struct sockaddr_storage Output_data_source_address; // Multicast output socket
uint64_t Output_metadata_packets;
char *Description;

struct sdrstate AirCD;
pthread_t Display_thread;
pthread_t Ncmd_thread;
FILE *Status;
char *Status_filename;
char *Pid_filename;
float Sampbuffer_i[BUFFERSIZE];
float Sampbuffer_q[BUFFERSIZE];
pthread_mutex_t Buf_mutex;
pthread_cond_t Buf_cond;
int Samp_wp;


uint64_t Commands;
struct state State[256];

// Gain and phase corrections. These will be updated every block
float gain_q = 1;
float gain_i = 1;
float secphi = 1;
float tanphi = 0;

void decode_airspy_commands(struct sdrstate *,unsigned char *,int);
void send_airspy_status(struct sdrstate *,int);
void do_airspy_agc(struct sdrstate *);
int rx_callback(airspy_transfer *);
void *display(void *);
void *ncmd(void *arg);
void errmsg(const char *,...);
double true_freq(uint64_t freq);


int main(int argc,char *argv[]){
  struct sdrstate * const sdr = &AirCD;

  Locale = getenv("LANG");
  if(Locale == NULL || strlen(Locale) == 0)
    Locale = "en_US.UTF-8";
  setlocale(LC_ALL,Locale);

  int c;
  while((c = getopt_long(argc,argv,Optstring,Options,NULL)) != -1){
    switch(c){
    case 'A':
      Default_mcast_iface = optarg;
      break;
    case 'D':
      Data_dest = optarg;
      break;
    case 'I':
      Device = strtol(optarg,NULL,0);
      break;
    case 'R':
      Metadata_dest = optarg;
      break;
    case 'S':
      Rtp.ssrc = strtol(optarg,NULL,0);
      break;
    case 'T':
      Mcast_ttl = strtol(optarg,NULL,0);
      break;
    case 'b':
      Blocksize = strtol(optarg,NULL,0);
      break;
    case 'c':
      Decimate = strtol(optarg,NULL,0);
      break;
    case 'd':
      Daemonize++;
      Status = NULL;
      break;
    case 'f':
      Frequency = strtod(optarg,NULL);
      break;
    case 'o':
      Offset = strtol(optarg,NULL,0);
      break;
    case 'r':
      Out_samprate = strtol(optarg,NULL,0);
      break;
    case 't':
      {
	int t = strtol(optarg,NULL,0);
	switch(t){
	case 12:
	  Rtp_type = IQ_PT12;
	  break;
	case 16:
	  Rtp_type = IQ_PT;
	  break;
	default:
	  fprintf(stderr,"Valid arguments to -t are 12 or 16\n");
	  break;
	}
      }
      break;
    case 'v':
      Verbose++;
      if(!Daemonize)
	Status = stderr;
      break;
    default:
    case '?':
      fprintf(stderr,"Unknown argument %c\n",c);
      break;
    }
  }
  Description = argv[optind];
  if(Daemonize){
    openlog("airspy",LOG_PID,LOG_DAEMON);

    // see if one is already running
    int r = asprintf(&Pid_filename,"%s%d/pid",Rundir,Device);
    if(r == -1){
      // Unlikely, but it makes the compiler happy
      errmsg("asprintf error");
      exit(1);
    }
    FILE *pidfile = fopen(Pid_filename,"r");
    if(pidfile){
      // pid file exists; read it and see if process exists
      int pid = 0;
      if(fscanf(pidfile,"%d",&pid) == 1 && (kill(pid,0) == 0 || errno != ESRCH)){
	// Already running; exit
	fclose(pidfile);
	errmsg("pid %d: daemon %d already running, quitting",getpid(),pid);
	exit(1);
      }
      fclose(pidfile); pidfile = NULL;
    }
    unlink(Pid_filename); // Remove any orphan
    pidfile = fopen(Pid_filename,"w");
    if(pidfile){
      int pid = getpid();
      fprintf(pidfile,"%d\n",pid);
      fclose(pidfile);
    }
    r = asprintf(&Status_filename,"%s%d/status",Rundir,Device);
    if(r == -1){
      // Unlikely, but it makes the compiler happy
      errmsg("asprintf error");
      exit(1);
    }

    unlink(Status_filename); // Remove any orphaned version
    Status = fopen(Status_filename,"w");
    if(Status == NULL){
      errmsg("Can't write %s: %s\n",Status_filename,strerror(errno));
    } else {
      setlinebuf(Status);
    }
  }
  
  ADC_samprate = Decimate * Out_samprate;
  Rate_factor = 1./(ADC_samprate * Power_alpha);
  Log_decimate = (int)round(log2(Decimate));
  if(1<<Log_decimate != Decimate){
    errmsg("Decimation ratios must currently be a power of 2\n");
    exit(1);
  }
  // Fold in scaling from float to short integer
  Filter_atten = 32767. * powf(.5, Log_decimate); // Compensate for +6dB gain in each decimation stage

  if(Decimate == 1){
    errmsg("No spectrum shift without decimation");
    Offset = 0; // No reason to offset when not decimating
  }
  if(2*sizeof(short) * Blocksize + 200 > Bufsize){
    Blocksize = (Bufsize - 200) / (2 * sizeof(short));
    errmsg("Blocksize reduced to %d",Blocksize);
  }

  // Set up RTP output socket
  Rtp_sock = setup_mcast(Data_dest,(struct sockaddr *)&Output_data_dest_address,1,Mcast_ttl,0);
  if(Rtp_sock == -1){
    errmsg("Can't create multicast socket: %s",strerror(errno));
    exit(1);
  }
  socklen_t len = sizeof(Output_data_source_address);
  getsockname(Rtp_sock,(struct sockaddr *)&Output_data_source_address,&len);
    
  int ret;
  if((ret = airspy_init()) != AIRSPY_SUCCESS){
    errmsg("airspy_init() failed: %s\n",airspy_error_name(ret));
    exit(1);
  }

#if 0
  // Enumerate devices - seems to require latest version of libusb
  airspy_list_devices(Serials,N_serials);

  if((ret = airspy_open_sn(sdr->device,Serials[Device])) != AIRSPY_SUCCESS){
    errmsg("airspy_open(%d) failed: %s\n",Device,airspy_error_name(ret));
    exit(1);
  }
#else
  if((ret = airspy_open(&sdr->device)) != AIRSPY_SUCCESS){
    errmsg("airspy_open(%d) failed: %s\n",Device,airspy_error_name(ret));    
    exit(1);
  }
    
#endif  

  // See how many there are
  ret = airspy_get_samplerates(sdr->device,Sample_rates,0);
  int number_sample_rates = Sample_rates[0];
  assert(ret == AIRSPY_SUCCESS);
  fprintf(stderr,"Number of sample rates: %d\n",number_sample_rates);

  ret = airspy_get_samplerates(sdr->device,Sample_rates,number_sample_rates);
  assert(ret == AIRSPY_SUCCESS);
  for(int n = 0; n < number_sample_rates; n++){
    fprintf(stderr,"%d\n",Sample_rates[n]);
    if(Sample_rates[n] < 1)
      break;
  }


  ret = airspy_set_samplerate(sdr->device,(uint32_t)ADC_samprate);
  assert(ret == AIRSPY_SUCCESS);
  sdr->status.samprate = Out_samprate;

  /* set filter here */

  sdr->status.lna_gain = 15;
  sdr->status.mixer_gain = 15;
  sdr->status.if_gain = 15;

  ret = airspy_set_lna_gain(sdr->device,sdr->status.lna_gain);
  assert(ret == AIRSPY_SUCCESS);
  ret = airspy_set_mixer_gain(sdr->device,sdr->status.mixer_gain);
  assert(ret == AIRSPY_SUCCESS);  
  ret = airspy_set_vga_gain(sdr->device,sdr->status.if_gain);
  assert(ret == AIRSPY_SUCCESS);

  uint64_t intfreq = sdr->status.frequency = Frequency;
  
  intfreq += Offset * ADC_samprate / 4; // Offset tune high by +Fs/4

  ret = airspy_set_freq(sdr->device,intfreq);
  assert(ret == AIRSPY_SUCCESS);

  pthread_mutex_init(&Buf_mutex,NULL);
  pthread_cond_init(&Buf_cond,NULL);


  time_t tt;
  time(&tt);
  if(Rtp.ssrc == 0)
    Rtp.ssrc = tt & 0xffffffff; // low 32 bits of clock time

  int sampsize = 0;
  switch(Rtp_type){
  case IQ_PT12:
    sampsize = 12;
    break;
  case IQ_PT:
    sampsize = 16;
    break;
  default:
    break;
  }

  // 7680 bytes data + 12 byte RTP + 8 byte UDP + 20 byte IP = 7720 bytes. MTU is 9000, overhead is 40 bytes
  // 7680 bytes data + 14 bytes ethernet + 20 byte IP + 8 byte UDP + 12 byte RTP = 7738 (ip length 7720)
  // Max sample bytes 8960: 2986.66667 @ 12 bits, 2240 @ 16 bits, 4480 @ 8 bits
  // 8958 bytes = 2986 samples
  // 8955 bytes = 2985 samples
  // 8952 bytes = 2984 samples
  // 6.144 MHz * 20 ms = 122,880 samples/20 ms frame
  // 3 * 2^N = 1536 samples
  // 5 * 2^N = 2560 samples; 48 packets/20 ms frame
  // 3*5 * 2^N = 1920

  errmsg("uid %d; device %d; dest %s; %d bit samples; blocksize %'d samples (%'d bytes, %'.3f ms); RTP SSRC %x; status file %s\n",
	 getuid(),
	 Device,
	 Data_dest,
	 sampsize,
	 Blocksize,
	 2*Blocksize*sampsize/8,
	 1000.*(float)Blocksize/Out_samprate,
	 Rtp.ssrc,
	 Status_filename);
  
  errmsg("A/D sample rate %'d Hz; filter bw %'d Hz; decimation ratio %d; output sample rate %'d Hz (%'d bits/sec); Offset %'+d\n",
	 ADC_samprate,
	 0,
	 Decimate,
	 Out_samprate,
	 Out_samprate * sampsize * 2,
	 Offset * ADC_samprate/4);


  ret = airspy_set_sample_type(sdr->device,AIRSPY_SAMPLE_FLOAT32_IQ);
  assert(ret == AIRSPY_SUCCESS);

  ret = airspy_start_rx(sdr->device,rx_callback,sdr);
  assert(ret == AIRSPY_SUCCESS);

  pthread_create(&Ncmd_thread,NULL,ncmd,sdr);
  signal(SIGPIPE,SIG_IGN);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  
  if(Status)
    pthread_create(&Display_thread,NULL,display,sdr);

  pthread_setname("aspy-decim");

  // Decimation filter states
  struct hb15_state hb15_state_real[Log_decimate];
  struct hb15_state hb15_state_imag[Log_decimate];
  memset(hb15_state_real,0,sizeof(hb15_state_real));
  memset(hb15_state_imag,0,sizeof(hb15_state_imag));
  float hb3state_real[Log_decimate];
  float hb3state_imag[Log_decimate];
  memset(hb3state_real,0,sizeof(hb3state_real));
  memset(hb3state_imag,0,sizeof(hb3state_imag));

#if 0
  // Verify SIMD alignment
  assert(((uint64_t)hb15_state_real & 15) == 0);
  assert(((uint64_t)hb15_state_imag & 15) == 0);
#endif

  // Initialize coefficients here!!!
  // As experiment, use Goodman/Carey "F8" 15-tap filter
  // Note word order in array -- [3] is closest to the center, [0] is on the tails
  for(int i=0; i<Log_decimate; i++){ // For each stage (h(0) is always unity, other h(n) are zero for even n)
    hb15_state_real[i].coeffs[3] = 490./802;
    hb15_state_imag[i].coeffs[3] = 490./802;
    hb15_state_real[i].coeffs[2] = -116./802;
    hb15_state_imag[i].coeffs[2] = -116./802;
    hb15_state_real[i].coeffs[1] = 33./802; 
    hb15_state_imag[i].coeffs[1] = 33./802;    
    hb15_state_real[i].coeffs[0] = -6./802; 
    hb15_state_imag[i].coeffs[0] = -6./802;    
  }
  struct timeval tp;
  gettimeofday(&tp,NULL);
  // Timestamp is in nanoseconds for futureproofing, but time of day is only available in microsec
  sdr->status.timestamp = ((tp.tv_sec - UNIX_EPOCH + GPS_UTC_OFFSET) * 1000000LL + tp.tv_usec) * 1000LL;

  int samp_rp = 0;

  while(1){
    struct rtp_header rtp;
    memset(&rtp,0,sizeof(rtp));
    rtp.version = RTP_VERS;
    rtp.type = Rtp_type;
    rtp.ssrc = Rtp.ssrc;
    rtp.seq = Rtp.seq++;
    rtp.timestamp = Rtp.timestamp;

    unsigned char buffer[Bufsize];
    unsigned char *dp = buffer;

    dp = hton_rtp(dp,&rtp);
    if(Rtp_type == IQ_PT)
      dp = hton_status(dp,&sdr->status); // old metadata header, will disappear someday

    float output_energy = 0;
    // NB: We assume that Decimate divides into BUFFERSIZE
    // They will since both are powers of 2
    // Wait for enough to be available to send a full packet
    pthread_mutex_lock(&Buf_mutex);
    while(((Samp_wp - samp_rp) & (BUFFERSIZE-1)) < Decimate * Blocksize)
      pthread_cond_wait(&Buf_cond,&Buf_mutex);
    pthread_mutex_unlock(&Buf_mutex);
    
    int remain = Blocksize;
    while(remain > 0){
      int chunk = remain;
      // Don't straddle the end of the circular sample buffer
      if(samp_rp + chunk * Decimate > BUFFERSIZE)
	chunk = (BUFFERSIZE - samp_rp) / Decimate;

      // Real channel in-place decimation
      // First stages can use simple, fast filter; later ones use slower filter
      // each stage is half the length of the previous one, and puts its output in the first half of its input buffer
      // so the final outputs are in the first 'chunk' elements of the buffer
      float *ip = &Sampbuffer_i[samp_rp];
      int j;
      for(j=Log_decimate-1;j>=stage_threshold;j--)
	hb3_block(&hb3state_real[j], ip, ip, chunk<<j);

      for(; j>=0;j--)
	hb15_block(&hb15_state_real[j], ip, ip, chunk<<j);

      // Imaginary channel decimation
      float *qp = &Sampbuffer_q[samp_rp];
      for(j=Log_decimate-1;j>=stage_threshold;j--)
	hb3_block(&hb3state_imag[j], qp, qp, chunk<<j);

      for(; j>=0;j--)
	hb15_block(&hb15_state_imag[j], qp, qp, chunk<<j);

      switch(Rtp_type){
      case IQ_PT:	  // 16-bit integers, little endian with metadata; will eventually become PCM_STEREO (10)
	{
	  short *sp = (short *)dp;
	  for(int i=0;i < chunk; i++){
	    float s = *ip++ * Filter_atten;
	    output_energy += s*s;
	    *sp++ = s; // Clip?

	    s = *qp++ * Filter_atten;
	    output_energy += s*s;
	    *sp++ = s; // Clip?
	  }
	  dp = (unsigned char *)sp;
	}
	break;
      case IQ_PT12:	  // 12-bit integers, packed big-endian, no metadata header
	{
	  for(int i=0;i < chunk; i++){
	    float s = *ip++ * Filter_atten;
	    output_energy += s*s;
	    short si = s; // Clip?

	    s = *qp++ * Filter_atten;
	    output_energy += s*s;
	    short sq = s; // Clip?

	    dp[0] = si >> 8;
	    dp[1] = (si & 0xf0) | ((sq >> 12) & 0xf);
	    dp[2] = sq >> 4;
	    dp += 3;
	  }
	}
	break;
      }
      samp_rp += chunk * Decimate;
      samp_rp &= (BUFFERSIZE-1);
      remain -= chunk;
    }
    // Remove scaling factor in power just once per block
    sdr->out_power = output_energy / (32767.0 * 32767.0 * Blocksize);
    if(send(Rtp_sock,buffer,dp - buffer,0) == -1){
      errmsg("send: %s",strerror(errno));
      // If we're sending to a unicast address without a listener, we'll get ECONNREFUSED
      // Sleep 1 sec to slow down the rate of these messages
      usleep(1000000);
    } else {
      Rtp.packets++;
      Rtp.bytes += Blocksize;
    }  
    Rtp.timestamp += Blocksize; // samples
#if 1
    // Get status timestamp from UNIX TOD clock -- but this might skew because of inexact sample rate
    gettimeofday(&tp,NULL);
    // Timestamp is in nanoseconds for futureproofing, but time of day is only available in microsec
    sdr->status.timestamp = ((tp.tv_sec - UNIX_EPOCH + GPS_UTC_OFFSET) * 1000000LL + tp.tv_usec) * 1000LL;
#else
    // Simply increment by number of samples
    // But what if we lose some? Then the clock will always be off
    sdr->status.timestamp += 1.e9 * Blocksize / ADC_samprate;
#endif
  }
  // Can't really get here
  close(Rtp_sock);
  airspy_close(sdr->device);
  airspy_exit();
  exit(0);

}

// Thread to send metadata and process commands
void *ncmd(void *arg){

  // Send status, process commands
  pthread_setname("arspy-cmd");
  assert(arg != NULL);
  struct sdrstate * const sdr = arg;
  
  memset(State,0,sizeof(State));

  // Set up status socket on port 5006
  Status_sock = setup_mcast(Metadata_dest,(struct sockaddr *)&Output_metadata_dest_address,1,Mcast_ttl,2); // For output

  if(Status_sock <= 0)
    return NULL; // Nothing to do

  // Set up new control socket on port 5006
  Nctl_sock = setup_mcast(NULL,(struct sockaddr *)&Output_metadata_dest_address,0,0,0); // For input
  if(Nctl_sock <= 0){
    close(Status_sock);
    return NULL;
  }


  int counter = 0;
  while(1){
    unsigned char buffer[Bufsize];
    memset(buffer,0,sizeof(buffer));
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100 ms

    if(setsockopt(Nctl_sock,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv))){
      perror("ncmd setsockopt");
      return NULL;
    }

    int length = recv(Nctl_sock,buffer,sizeof(buffer),0);
    if(length > 0){
      // Parse entries
      unsigned char *cp = buffer;

      int cr = *cp++; // Command/response
      if(cr == 0)
	continue; // Ignore our own status messages
      Commands++;
      decode_airspy_commands(sdr,cp,length-1);
    }      
    Output_metadata_packets++;
    send_airspy_status(sdr,(counter == 0));
    if(counter-- <= 0)
      counter = 10;
		       
    do_airspy_agc(sdr);
  }
}

// Status display thread
void *display(void *arg){
  assert(arg != NULL);
  struct sdrstate *sdr = (struct sdrstate *)arg;

  pthread_setname("airspy-disp");

  fprintf(Status,"               |---Gains dB---|      |----Levels dB --|   |---------Errors---------|           clips\n");
  fprintf(Status,"Frequency      LNA  mixer bband          RF   A/D   Out     DC-I   DC-Q  phase  gain\n");
  fprintf(Status,"Hz                                           dBFS  dBFS                    deg    dB\n");   

  off_t stat_point = ftello(Status);
  // End lines with return when writing to terminal, newlines when writing to status file
  char   eol = stat_point == -1 ? '\r' : '\n';
  while(1){

    float powerdB = 10*log10f(sdr->in_power);

    if(stat_point != -1)
      fseeko(Status,stat_point,SEEK_SET);
    
    fprintf(Status,"%'-15.0lf%3d%7d%6d%'12.1f%'6.1f%'6.1f%9.4f%7.4f%7.2f%6.2f%'16d    %c",
	    sdr->status.frequency,
	    sdr->status.lna_gain,	    
	    sdr->status.mixer_gain,
	    sdr->status.if_gain,
	    powerdB - (sdr->status.lna_gain + sdr->status.mixer_gain + sdr->status.if_gain),
	    powerdB,
	    10*log10f(sdr->out_power),
	    crealf(sdr->DC),
	    cimagf(sdr->DC),
	    (180/M_PI) * asin(sdr->sinphi),
	    10*log10(sdr->imbalance),
	    sdr->clips,
	    eol);
    fflush(Status);
    usleep(100000); // 10 Hz
  }
  return NULL;
}

void decode_airspy_commands(struct sdrstate *sdr,unsigned char *buffer,int length){
  unsigned char *cp = buffer;


  while(cp - buffer < length){
    int ret __attribute__((unused)); // Won't be used when asserts are disabled
    enum status_type type = *cp++; // increment cp to length field
    
    if(type == EOL)
      break; // End of list
    
    unsigned int optlen = *cp++;
    if(cp - buffer + optlen >= length)
      break; // Invalid length
    
    uint64_t intfreq;
    double f,tf;
    
    switch(type){
    case EOL: // Shouldn't get here
      break;
    case CALIBRATE:
      sdr->calibration = decode_double(cp,optlen);
      break;
    case RADIO_FREQUENCY:
      f = decode_double(cp,optlen) + Offset * ADC_samprate/4; // Offset tune by +Fs/4
      intfreq = round(f / (1 + sdr->calibration));
      ret = airspy_set_freq(sdr->device,intfreq);
      assert(ret == AIRSPY_SUCCESS);
      tf = true_freq(intfreq) - Offset * ADC_samprate/4;
      sdr->status.frequency = tf * (1 + sdr->calibration);
      break;
    case LNA_GAIN:	// Fill this in later
      sdr->status.lna_gain = decode_int(cp,optlen);
      break;
    case MIXER_GAIN:	// Fill this in later
      sdr->status.mixer_gain = decode_int(cp,optlen);
      break;
    case IF_GAIN:	// Fill this in later
      sdr->status.if_gain = decode_int(cp,optlen);
      break;
    default: // Ignore all others
      break;
    }
    cp += optlen;
  }    
}  

void send_airspy_status(struct sdrstate *sdr,int full){
  unsigned char packet[2048],*bp;
  memset(packet,0,sizeof(packet));
  bp = packet;
  
  *bp++ = 0;   // Command/response = response
  
  encode_int32(&bp,COMMAND_TAG,sdr->command_tag);
  encode_int64(&bp,COMMANDS,Commands);
  
  struct timeval tp;
  gettimeofday(&tp,NULL);
  // Timestamp is in nanoseconds for futureproofing, but time of day is only available in microsec
  long long timestamp = ((tp.tv_sec - UNIX_EPOCH + GPS_UTC_OFFSET) * 1000000LL + tp.tv_usec) * 1000LL;
  encode_int64(&bp,GPS_TIME,timestamp);

  if(Description)
    encode_string(&bp,DESCRIPTION,Description,strlen(Description));

  // Source address we're using to send data
  encode_socket(&bp,OUTPUT_DATA_SOURCE_SOCKET,&Output_data_source_address);
  // Where we're sending output
  encode_socket(&bp,OUTPUT_DATA_DEST_SOCKET,&Output_data_dest_address);
  encode_int32(&bp,OUTPUT_SSRC,Rtp.ssrc);
  encode_byte(&bp,OUTPUT_TTL,Mcast_ttl);
  encode_int32(&bp,INPUT_SAMPRATE,ADC_samprate);  // This should be the actual A/D sample rate, which will be higher
  encode_int32(&bp,OUTPUT_SAMPRATE,Out_samprate);
  encode_int64(&bp,OUTPUT_DATA_PACKETS,Rtp.packets);
  encode_int64(&bp,OUTPUT_METADATA_PACKETS,Output_metadata_packets);
  
  // Front end
  encode_double(&bp,AD_LEVEL,power2dB(sdr->in_power));
  encode_double(&bp,CALIBRATE,sdr->calibration);
  encode_byte(&bp,LNA_GAIN,sdr->status.lna_gain);
  encode_byte(&bp,MIXER_GAIN,sdr->status.mixer_gain);
  encode_byte(&bp,IF_GAIN,sdr->status.if_gain);
  encode_float(&bp,DC_I_OFFSET,crealf(sdr->DC));
  encode_float(&bp,DC_Q_OFFSET,cimagf(sdr->DC));
  encode_float(&bp,IQ_IMBALANCE,power2dB(sdr->imbalance));
  encode_float(&bp,IQ_PHASE,sdr->sinphi);
  encode_byte(&bp,DIRECT_CONVERSION,Offset == 0); // Direct conversion if offset == 0
  
  // Tuning
  encode_double(&bp,RADIO_FREQUENCY,sdr->status.frequency);

  // Filtering
  encode_float(&bp,LOW_EDGE,-0.47 * Out_samprate); // Should look at the actual filter curves
  encode_float(&bp,HIGH_EDGE,+0.47 * Out_samprate);
  
  encode_float(&bp,OUTPUT_LEVEL,power2dB(sdr->out_power));
  
  float analog_gain = sdr->status.mixer_gain + sdr->status.if_gain + sdr->status.lna_gain;
  encode_float(&bp,GAIN,analog_gain);
  encode_byte(&bp,DEMOD_TYPE,0); // actually LINEAR_MODE
  encode_int32(&bp,OUTPUT_CHANNELS,2);


  encode_eol(&bp);
  assert(bp - packet < sizeof(packet));
  
  int len = compact_packet(&State[0],packet,full);
  send(Status_sock,packet,len,0);
}


int rotate_phase = 0;

complex float rotate_phasor = 1;


// Callback called with incoming receiver data from A/D
int rx_callback(airspy_transfer *transfer){

  struct sdrstate *sdr = &AirCD;

  int samples = transfer->sample_count;
  int remain = samples;
  float *dp = transfer->samples;

  complex float samp_sum = 0;
  float i_energy=0,q_energy=0;
  float dotprod = 0;                           // sum of I*Q, for phase balance

  while(remain-- > 0){
    complex float samp;

    __real__ samp = *dp++;
    __imag__ samp = *dp++;

    if(__imag__ samp < -1){
      sdr->clips++;
      __imag__ samp = -1;
    }
    if(__real__ samp < -1){
      sdr->clips++;
      __real__ samp = -1;
    }
    samp_sum += samp;

    // remove DC offset (which can be fractional)
    samp -= sdr->DC;
    
    // Must correct gain and phase before frequency shift
    // accumulate I and Q energies before gain correction
    i_energy += crealf(samp) * crealf(samp);
    q_energy += cimagf(samp) * cimagf(samp);
    
    // Balance gains, keeping constant total energy
    __real__ samp *= gain_i;
    __imag__ samp *= gain_q;
    
    // Accumulate phase error
    dotprod += crealf(samp) * cimagf(samp);

    // Correct phase
    __imag__ samp = secphi * cimagf(samp) - tanphi * crealf(samp);
    
#if 0
    // On Atom CPU, seems slightly slower than swapping/flipping code below
    samp *= rotate_phasor;
    rotate_phasor *= _Complex_I;
    Sampbuffer_i[Samp_wp] = __real__ samp;
    Sampbuffer_q[Samp_wp] = __imag__ samp;

#else
    // Optionally increase frequency by Fs/4 to compensate for tuner being high by Fs/4
    switch(rotate_phase){
    default:
    case 0:
      Sampbuffer_i[Samp_wp] = __real__ samp;
      Sampbuffer_q[Samp_wp] = __imag__ samp;
      break;
    case 1:
      Sampbuffer_i[Samp_wp] = - __imag__ samp;
      Sampbuffer_q[Samp_wp] = __real__ samp;
      break;
    case 2:
      Sampbuffer_i[Samp_wp] = - __real__ samp;
      Sampbuffer_q[Samp_wp] = - __imag__ samp;
      break;
    case 3:
      Sampbuffer_i[Samp_wp] = __imag__ samp;
      Sampbuffer_q[Samp_wp] = - __real__ samp;
      break;
    }
    rotate_phase += Offset;
    rotate_phase &= 3; // Modulo 4
#endif

    Samp_wp = (Samp_wp + 1) & (BUFFERSIZE-1);
  }
  pthread_cond_broadcast(&Buf_cond); // Wake him up only after we're done
  // Update every block
  // estimates of DC offset, signal powers and phase error
  sdr->DC += DC_alpha * (samp_sum/samples - sdr->DC);
  float block_energy = i_energy + q_energy; // Normalize for complex pairs
  if(block_energy > 0){ // Avoid divisions by 0, etc
    sdr->in_power = block_energy/samples; // Average A/D output power per channel  
    sdr->imbalance += Rate_factor * samples * ((i_energy / q_energy) - sdr->imbalance);
    float dpn = 2 * dotprod / block_energy;
    sdr->sinphi += Rate_factor * samples * (dpn - sdr->sinphi);
    gain_q = sqrtf(0.5 * (1 + sdr->imbalance));
    gain_i = sqrtf(0.5 * (1 + 1./sdr->imbalance));
    secphi = 1/sqrtf(1 - sdr->sinphi * sdr->sinphi); // sec(phi) = 1/cos(phi)
    tanphi = sdr->sinphi * secphi;             // tan(phi) = sin(phi) * sec(phi) = sin(phi)/cos(phi)
  }
  return 0;
}

void do_airspy_agc(struct sdrstate *sdr){
  assert(sdr != NULL);
    
  float powerdB = 10*log10f(sdr->in_power);
  int change;
  if(powerdB > AGC_upper)
    change = AGC_upper - powerdB;
  else if(powerdB < AGC_lower)
    change = AGC_lower - powerdB;
  else
    return;
    
  int ret __attribute__((unused)) = AIRSPY_SUCCESS; // Won't be used when asserts are disabled
  // Increase gain, LNA first, then mixer, and finally IF
  if(change > 0){
    int lna_change = min(change,15 - sdr->status.lna_gain);
    sdr->status.lna_gain += lna_change;
    change -= lna_change;
    ret = airspy_set_lna_gain(sdr->device,sdr->status.lna_gain);
    assert(ret == AIRSPY_SUCCESS);
    if(change > 0){
      int mixer_change = min(change,15 - sdr->status.mixer_gain);
      sdr->status.mixer_gain += mixer_change;
      change -= mixer_change;
      ret = airspy_set_mixer_gain(sdr->device,sdr->status.mixer_gain);
      assert(ret == AIRSPY_SUCCESS);
      if(change > 0){
	int if_change = min(change,15 - sdr->status.if_gain);
	sdr->status.if_gain += if_change;
	change -= if_change;
	ret = airspy_set_vga_gain(sdr->device,sdr->status.if_gain);
	assert(ret == AIRSPY_SUCCESS);
      }
    }
  } else if(change < 0){
    // Decrease IF gain first
    change = -change;
    int if_change = min(change,(int)sdr->status.if_gain);
    sdr->status.if_gain -= if_change;
    change -= if_change;
    ret = airspy_set_vga_gain(sdr->device,sdr->status.if_gain);
    assert(ret == AIRSPY_SUCCESS);
    if(change > 0){
      char mixer_change = min(change,(int)sdr->status.mixer_gain);
      sdr->status.mixer_gain -= mixer_change;
      change -= mixer_change;
      ret = airspy_set_mixer_gain(sdr->device,sdr->status.mixer_gain);
      assert(ret == AIRSPY_SUCCESS);
      if(change > 0){
	char lna_change = min(change,(int)sdr->status.lna_gain);
	sdr->status.lna_gain -= lna_change;
	change -= sdr->status.lna_gain;
	ret = airspy_set_lna_gain(sdr->device,sdr->status.lna_gain);	
	assert(ret == AIRSPY_SUCCESS);
      }
    }
  }
}

// Write this later
double true_freq(uint64_t intfreq){
  return intfreq;
}



void closedown(int a){
  errmsg("caught signal %d: %s\n",a,strsignal(a));
  airspy_close(AirCD.device);
  airspy_exit();

  if(a == SIGTERM) // sent by systemd when shutting down. Return success
    exit(0);
  exit(1);
}

void errmsg(const char *fmt,...){
  va_list ap;

  va_start(ap,fmt);

  if(Daemonize){
    vsyslog(LOG_INFO,fmt,ap);
  } else {
    vfprintf(stderr,fmt,ap);
    fflush(stderr);
  }
  va_end(ap);
}
