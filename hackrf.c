// $Id: hackrf.c,v 1.25 2018/12/13 09:47:57 karn Exp karn $
// Read from HackRF
// Multicast raw 8-bit I/Q samples
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
#include <libhackrf/hackrf.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>
#include <syslog.h>
#include <sys/stat.h>

#include "sdr.h"
#include "misc.h"
#include "multicast.h"
#include "decimate.h"
#include "status.h"
#include "dsp.h"

struct sdrstate {
  hackrf_device *device;    // Opaque pointer
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
int Out_samprate = 192000;
int Decimate = 64;
int Log_decimate = 6; // Computed from Decimate
const float SCALE8 = 1./127.;   // Scale 8-bit samples to unity range floats
float const DC_alpha = 1.0e-7;  // high pass filter coefficient for DC offset estimates, per sample
float const Power_alpha= 1.0; // time constant (seconds) for smoothing power and I/Q imbalance estimates
char *Rundir = "/run/hackrf"; // Where 'status' and 'pid' get written
float Filter_atten = 1;
int const stage_threshold = 8; // point at which to switch to filter f8
#define BUFFERSIZE  (1<<19) // Upcalls seem to be 256KB; don't make too big or we may blow out of the cache

// Variables set by command line options
int Blocksize = 350;
int Device = 0;      // Which of several to use
char *Locale;
int Offset=1;     // Default to offset high by +Fs/4 downconvert in software to avoid DC
int Daemonize = 0;
int Mcast_ttl = 1; // Don't send fast IQ streams beyond the local network by default
char *Data_dest = "239.1.6.10";
char *Metadata_dest = "239.1.6.1"; // Default for testing
struct sockaddr_storage Output_metadata_dest_address;

// Global variables
struct rtp_state Rtp;
int Rtp_sock;     // Socket handle for sending real time stream
int Nctl_sock;    // Socket handle for incoming commands
int Status_sock;  // Socket handle for outgoing status messages
struct sockaddr_storage Output_data_dest_address; // Multicast output socket
struct sockaddr_storage Output_data_source_address; // Multicast output socket
uint64_t Output_metadata_packets;
char *Description;

struct sdrstate HackCD;
pthread_t Display_thread;
pthread_t Ncmd_thread;
FILE *Status;
char *Status_filename;
char *Pid_filename;
complex float Sampbuffer[BUFFERSIZE];
pthread_mutex_t Buf_mutex;
pthread_cond_t Buf_cond;
int Samp_wp;
int Samp_rp;

uint64_t Commands;
struct state State[256];

// Gain and phase corrections. These will be updated every block
float gain_q = 1;
float gain_i = 1;
float secphi = 1;
float tanphi = 0;

void decode_hackrf_commands(struct sdrstate *,unsigned char *,int);
void send_hackrf_status(struct sdrstate *,int);
void do_hackrf_agc(struct sdrstate *);
int rx_callback(hackrf_transfer *);
void *display(void *);
void *ncmd(void *arg);
void errmsg(const char *,...);


int main(int argc,char *argv[]){
  struct sdrstate * const sdr = &HackCD;

  Locale = getenv("LANG");
  if(Locale == NULL || strlen(Locale) == 0)
    Locale = "en_US.UTF-8";

  int c;
  while((c = getopt(argc,argv,"D:I:dvl:b:R:T:o:r:S:")) != -1){
    switch(c){
    case 'd':
      Daemonize++;
      Status = NULL;
      break;
    case 'o':
      Offset = strtol(optarg,NULL,0);
      break;
    case 'r':
      Out_samprate = strtol(optarg,NULL,0);
      break;
    case 'R':
      Metadata_dest = optarg;
      break;
    case 'D':
      Data_dest = optarg;
      break;
    case 'I':
      Device = strtol(optarg,NULL,0);
      break;
    case 'v':
      if(!Daemonize)
	Status = stderr;
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
    case 'S':
      Rtp.ssrc = strtol(optarg,NULL,0);
      break;
    default:
    case '?':
      fprintf(stderr,"Unknown argument %c\n",c);
      break;
    }
  }
  Description = argv[optind];
  setlocale(LC_ALL,Locale);
  if(Daemonize){
    openlog("hackrf",LOG_PID,LOG_DAEMON);

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
  } else {
    Status = stderr; // Write status to stderr when running in foreground
  }
  
  ADC_samprate = Decimate * Out_samprate;
  Log_decimate = (int)round(log2(Decimate));
  if(1<<Log_decimate != Decimate){
    errmsg("Decimation ratios must currently be a power of 2\n");
    exit(1);
  }
  Filter_atten = powf(.5, Log_decimate); // Compensate for +6dB gain in each decimation stage

  // Set up RTP output socket
  Rtp_sock = setup_mcast(Data_dest,(struct sockaddr *)&Output_data_dest_address,1,Mcast_ttl,0);
  if(Rtp_sock == -1){
    errmsg("Can't create multicast socket: %s",strerror(errno));
    exit(1);
  }
  socklen_t len = sizeof(Output_data_source_address);
  getsockname(Rtp_sock,(struct sockaddr *)&Output_data_source_address,&len);
    
  int ret;
  if((ret = hackrf_init()) != HACKRF_SUCCESS){
    errmsg("hackrf_init() failed: %s\n",hackrf_error_name(ret));
    exit(1);
  }
  // Enumerate devices
  hackrf_device_list_t *dlist = hackrf_device_list();

  if((ret = hackrf_device_list_open(dlist,Device,&sdr->device)) != HACKRF_SUCCESS){
    errmsg("hackrf_open(%d) failed: %s\n",Device,hackrf_error_name(ret));
    exit(1);
  }
  hackrf_device_list_free(dlist); dlist = NULL;

  ret = hackrf_set_sample_rate(sdr->device,(double)ADC_samprate);
  assert(ret == HACKRF_SUCCESS);
  sdr->status.samprate = Out_samprate;

  uint32_t bw = hackrf_compute_baseband_filter_bw_round_down_lt(ADC_samprate);
  ret = hackrf_set_baseband_filter_bandwidth(sdr->device,bw);
  assert(ret == HACKRF_SUCCESS);

  // NOTE: what we call mixer gain, they call lna gain
  // What we call lna gain, they call antenna enable
  sdr->status.lna_gain = 14;
  sdr->status.mixer_gain = 24;
  sdr->status.if_gain = 20;

  ret = hackrf_set_antenna_enable(sdr->device,sdr->status.lna_gain ? 1 : 0);
  assert(ret == HACKRF_SUCCESS);
  ret = hackrf_set_lna_gain(sdr->device,sdr->status.mixer_gain);
  assert(ret == HACKRF_SUCCESS);
  ret = hackrf_set_vga_gain(sdr->device,sdr->status.if_gain);
  assert(ret == HACKRF_SUCCESS);

  uint64_t intfreq = sdr->status.frequency = 146000000;
  
  intfreq += Offset * ADC_samprate / 4; // Offset tune high by +Fs/4

  ret = hackrf_set_freq(sdr->device,intfreq);
  assert(ret == HACKRF_SUCCESS);

  pthread_mutex_init(&Buf_mutex,NULL);
  pthread_cond_init(&Buf_cond,NULL);


  time_t tt;
  time(&tt);
  if(Rtp.ssrc == 0)
    Rtp.ssrc = tt & 0xffffffff; // low 32 bits of clock time
  errmsg("uid %d; device %d; dest %s; blocksize %d; RTP SSRC %lx; status file %s\n",getuid(),Device,Data_dest,Blocksize,Rtp.ssrc,Status_filename);
  errmsg("A/D sample rate %'d Hz; decimation ratio %d; output sample rate %'d Hz; Offset %'+d\n",
	 ADC_samprate,Decimate,Out_samprate,Offset * ADC_samprate/4);

  ret = hackrf_start_rx(sdr->device,rx_callback,&HackCD);
  assert(ret == HACKRF_SUCCESS);

  pthread_create(&Ncmd_thread,NULL,ncmd,&HackCD);
  signal(SIGPIPE,SIG_IGN);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        
  
  if(Status)
    pthread_create(&Display_thread,NULL,display,&HackCD);

  int rotate_phase = 0;

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

  while(1){
    struct rtp_header rtp;
    memset(&rtp,0,sizeof(rtp));
    rtp.version = RTP_VERS;
    rtp.type = IQ_PT;
    rtp.ssrc = Rtp.ssrc;
    rtp.seq = Rtp.seq++;
    rtp.timestamp = Rtp.timestamp;

    unsigned char buffer[200+2*Blocksize*sizeof(short)];
    unsigned char *dp = buffer;

    dp = hton_rtp(dp,&rtp);
    dp = hton_status(dp,&sdr->status);

    // Wait for enough to be available
    pthread_mutex_lock(&Buf_mutex);
    while(1){
      int avail = (Samp_wp - Samp_rp) & (BUFFERSIZE-1);
      if(avail >= Blocksize*Decimate)
	break;
      pthread_cond_wait(&Buf_cond,&Buf_mutex);
    }
    pthread_mutex_unlock(&Buf_mutex);
    
    float workblock_real[Decimate*Blocksize];    // Hold input to first decimator, half used on each filter call
    float workblock_imag[Decimate*Blocksize];

    // Load first stage with corrected samples
    int loop_limit = Decimate * Blocksize;
    for(int i=0; i<loop_limit; i++){
      complex float samp = Sampbuffer[Samp_rp++];
      float samp_i = crealf(samp);
      float samp_q = cimagf(samp);
      Samp_rp &= (BUFFERSIZE-1); // Assume even buffer size

      // Increase frequency by Fs/4 to compensate for tuner being high by Fs/4
      switch(rotate_phase){
      default:
      case 0:
	workblock_real[i] = samp_i;
	workblock_imag[i] = samp_q;
	break;
      case 1:
	workblock_real[i] = -samp_q;
	workblock_imag[i] = samp_i;
	break;
      case 2:
	workblock_real[i] = -samp_i;
	workblock_imag[i] = -samp_q;
	break;
      case 3:
	workblock_real[i] = samp_q;
	workblock_imag[i] = -samp_i;
	break;
      }
      rotate_phase += Offset;
      rotate_phase &= 3; // Modulo 4
    }

    // Real channel decimation
    // First stages can use simple, fast filter; later ones use slower filter
    int j;
    for(j=Log_decimate-1;j>=stage_threshold;j--)
      hb3_block(&hb3state_real[j],workblock_real,workblock_real,(1<<j)*Blocksize);

    for(; j>=0;j--)
      hb15_block(&hb15_state_real[j],workblock_real,workblock_real,(1<<j)*Blocksize);

    float output_energy = 0;
    signed short *up = (signed short *)dp;
    loop_limit = Blocksize;
    for(int j=0;j<loop_limit;j++){
      float s = workblock_real[j] * Filter_atten;
      output_energy += s*s;
      *up++ = (short)round(32767 * s);
      up++;
    }

    // Imaginary channel decimation
    for(j=Log_decimate-1;j>=stage_threshold;j--)
      hb3_block(&hb3state_imag[j],workblock_imag,workblock_imag,(1<<j)*Blocksize);

    for(; j>=0;j--)
      hb15_block(&hb15_state_imag[j],workblock_imag,workblock_imag,(1<<j)*Blocksize);

    // Interleave imaginary samples following real
    up = (signed short *)dp;
    loop_limit = Blocksize;
    for(int j=0;j<loop_limit;j++){
      float s = workblock_imag[j] * Filter_atten;
      output_energy += s*s;
      up++;
      *up++ = (short)round(32767 * s);
    }

    sdr->out_power = output_energy / Blocksize;
    dp = (unsigned char *)up;
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
  hackrf_close(sdr->device);
  hackrf_exit();
  exit(0);

}

// Thread to send metadata and process commands
void *ncmd(void *arg){

  // Send status, process commands
  pthread_setname("hackrf-cmd");
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
    unsigned char buffer[8192];
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
      decode_hackrf_commands(sdr,cp,length-1);
    }      
    Output_metadata_packets++;
    send_hackrf_status(sdr,(counter == 0));
    if(counter-- <= 0)
      counter = 10;
		       
    do_hackrf_agc(sdr);
  }
}

// Status display thread
void *display(void *arg){
  assert(arg != NULL);
  struct sdrstate *sdr = (struct sdrstate *)arg;

  pthread_setname("hackrf-disp");

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

void decode_hackrf_commands(struct sdrstate *sdr,unsigned char *buffer,int length){
  unsigned char *cp = buffer;

  while(cp - buffer < length){
    int ret __attribute__((unused)); // Won't be used when asserts are disabled
    enum status_type type = *cp++; // increment cp to length field
    
    if(type == EOL)
      break; // End of list
    
    unsigned int optlen = *cp++;
    if(cp - buffer + optlen >= length)
      break; // Invalid length
    
    switch(type){
    case EOL: // Shouldn't get here
      break;
    case CALIBRATE:
      sdr->calibration = decode_double(cp,optlen);
      break;
    case RADIO_FREQUENCY:
      sdr->status.frequency = decode_double(cp,optlen);
      uint64_t intfreq = sdr->status.frequency;
      intfreq += Offset * ADC_samprate/4; // Offset tune by +Fs/4
      ret = hackrf_set_freq(sdr->device,intfreq);
      assert(ret == HACKRF_SUCCESS);
      
      sdr->status.frequency = round(sdr->status.frequency/ (1 + sdr->calibration));
      // LNA gain is frequency-dependent
      if(sdr->status.lna_gain){
	if(sdr->intfreq >= 420e6)
	  sdr->status.lna_gain = 7;
	else
	  sdr->status.lna_gain = 24;
      }
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

void send_hackrf_status(struct sdrstate *sdr,int full){
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

  // Source address we're using to send data
  encode_socket(&bp,OUTPUT_DATA_SOURCE_SOCKET,&Output_data_source_address);
  // Where we're sending output
  encode_socket(&bp,OUTPUT_DATA_DEST_SOCKET,&Output_data_dest_address);
  encode_int32(&bp,OUTPUT_SSRC,Rtp.ssrc);
  encode_byte(&bp,OUTPUT_TTL,Mcast_ttl);
  encode_int32(&bp,INPUT_SAMPRATE,Out_samprate);  // This should be the actual A/D sample rate, which will be higher
  encode_int32(&bp,OUTPUT_SAMPRATE,Out_samprate);
  encode_int64(&bp,OUTPUT_DATA_PACKETS,Rtp.packets);
  encode_int64(&bp,OUTPUT_METADATA_PACKETS,Output_metadata_packets);
  
  // Tuning
  encode_double(&bp,RADIO_FREQUENCY,sdr->status.frequency);
  encode_double(&bp,CALIBRATE,sdr->calibration);
  
  // Front end
  encode_byte(&bp,LNA_GAIN,sdr->status.lna_gain);
  encode_byte(&bp,MIXER_GAIN,sdr->status.mixer_gain);
  encode_byte(&bp,IF_GAIN,sdr->status.if_gain);
  encode_float(&bp,DC_I_OFFSET,crealf(sdr->DC));
  encode_float(&bp,DC_Q_OFFSET,cimagf(sdr->DC));
  encode_float(&bp,IQ_PHASE,sdr->sinphi);
  encode_float(&bp,IQ_IMBALANCE,sdr->imbalance);
  
  // Filtering
  encode_float(&bp,LOW_EDGE,-90e3);
  encode_float(&bp,HIGH_EDGE,+90e3);
  
  // Signals - these ALWAYS change
  encode_float(&bp,BASEBAND_POWER,power2dB(sdr->in_power));
  encode_float(&bp,IF_POWER,power2dB(sdr->in_power));   // Same, since there's no filtering
  
  encode_byte(&bp,DEMOD_TYPE,0); // actually LINEAR_MODE
  encode_int32(&bp,OUTPUT_CHANNELS,2);
  if(Description)
    encode_string(&bp,DESCRIPTION,Description,strlen(Description));

  encode_eol(&bp);
  assert(bp - packet < sizeof(packet));
  
  int len = compact_packet(&State[0],packet,full);
  send(Status_sock,packet,len,0);
}


// Callback called with incoming receiver data from A/D
int rx_callback(hackrf_transfer *transfer){

  struct sdrstate *sdr = &HackCD;

  int remain = transfer->valid_length; // Count of individual samples; divide by 2 to get complex samples
  int samples = remain / 2;            // Complex samples
  unsigned char *dp = transfer->buffer;

  complex float samp_sum = 0;
  float i_energy=0,q_energy=0;
  float dotprod = 0;                           // sum of I*Q, for phase balance
  float rate_factor = 1./(ADC_samprate * Power_alpha);

  while(remain > 0){
    complex float samp;
    int isamp_i = (char)*dp++;
    int isamp_q = (char)*dp++;
    remain -= 2;

    if(isamp_q == -128){
      sdr->clips++;
      isamp_q = -127;
    }
    if(isamp_i == -128){
      sdr->clips++;
      isamp_i = -127;
    }
    samp = CMPLXF(isamp_i,isamp_q) * SCALE8; // -1.0 to +1.0

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
    
    Sampbuffer[Samp_wp] = samp;
    Samp_wp = (Samp_wp + 1) & (BUFFERSIZE-1);
  }
  pthread_cond_signal(&Buf_cond); // Wake him up only after we're done
  // Update every block
  // estimates of DC offset, signal powers and phase error
  sdr->DC += DC_alpha * (samp_sum - samples*sdr->DC);
  float block_energy = i_energy + q_energy; // Normalize for complex pairs
  if(block_energy > 0){ // Avoid divisions by 0, etc
    //sdr->in_power += rate_factor * (block_energy - samples*sdr->in_power); // Average A/D output power per channel  
    sdr->in_power = block_energy/samples; // Average A/D output power per channel  
    sdr->imbalance += rate_factor * samples * ((i_energy / q_energy) - sdr->imbalance);
    float dpn = 2 * dotprod / block_energy;
    sdr->sinphi += rate_factor  * samples * (dpn - sdr->sinphi);
    gain_q = sqrtf(0.5 * (1 + sdr->imbalance));
    gain_i = sqrtf(0.5 * (1 + 1./sdr->imbalance));
    secphi = 1/sqrtf(1 - sdr->sinphi * sdr->sinphi); // sec(phi) = 1/cos(phi)
    tanphi = sdr->sinphi * secphi;                     // tan(phi) = sin(phi) * sec(phi) = sin(phi)/cos(phi)
  }
  return 0;
}

void do_hackrf_agc(struct sdrstate *sdr){
  assert(sdr != NULL);
    
  float powerdB = 10*log10f(sdr->in_power);
  int change;
  if(powerdB > AGC_upper)
    change = AGC_upper - powerdB;
  else if(powerdB < AGC_lower)
    change = AGC_lower - powerdB;
  else
    return;
    
  int ret __attribute__((unused)) = HACKRF_SUCCESS; // Won't be used when asserts are disabled
  if(change > 0){
    // Increase gain, LNA first, then mixer, and finally IF
    if(change >= 14 && sdr->status.lna_gain < 14){
      sdr->status.lna_gain = 14;
      change -= 14;
      ret = hackrf_set_antenna_enable(sdr->device,sdr->status.lna_gain ? 1 : 0);
      assert(ret == HACKRF_SUCCESS);
    }
    int old_mixer_gain = sdr->status.mixer_gain;
    int new_mixer_gain = min(40,old_mixer_gain + 8*(change/8));
    if(new_mixer_gain != old_mixer_gain){
      sdr->status.mixer_gain = new_mixer_gain;
      change -= new_mixer_gain - old_mixer_gain;
      ret = hackrf_set_lna_gain(sdr->device,sdr->status.mixer_gain);
      assert(ret == HACKRF_SUCCESS);
    }
    int old_if_gain = sdr->status.if_gain;
    int new_if_gain = min(62,old_if_gain + 2*(change/2));
    if(new_if_gain != old_if_gain){
      sdr->status.if_gain = new_if_gain;
      change -= new_if_gain - old_if_gain;
      ret = hackrf_set_vga_gain(sdr->device,sdr->status.if_gain);
      assert(ret == HACKRF_SUCCESS);
    }
  } else if(change < 0){
    // Reduce gain (IF first), start counter
    int old_if_gain = sdr->status.if_gain;
    int new_if_gain = max(0,old_if_gain + 2*(change/2));
    if(new_if_gain != old_if_gain){
      sdr->status.if_gain = new_if_gain;
      change -= new_if_gain - old_if_gain;
      ret = hackrf_set_vga_gain(sdr->device,sdr->status.if_gain);
      assert(ret == HACKRF_SUCCESS);
    }
    int old_mixer_gain = sdr->status.mixer_gain;
    int new_mixer_gain = max(0,old_mixer_gain + 8*(change/8));
    if(new_mixer_gain != old_mixer_gain){
      sdr->status.mixer_gain = new_mixer_gain;
      change -= new_mixer_gain - old_mixer_gain;
      ret = hackrf_set_lna_gain(sdr->device,sdr->status.mixer_gain);
      assert(ret == HACKRF_SUCCESS);
    }
    int old_lna_gain = sdr->status.lna_gain;
    int new_lna_gain = max(0,old_lna_gain + 14*(change/14));
    if(new_lna_gain != old_lna_gain){
      sdr->status.lna_gain = new_lna_gain;
      change -= new_lna_gain - old_lna_gain;
      ret = hackrf_set_antenna_enable(sdr->device,sdr->status.lna_gain ? 1 : 0);
      assert(ret == HACKRF_SUCCESS);
    }
  }
}


#if 0

double  rffc5071_freq(uint16_t);
uint32_t max2837_freq(uint32_t);


#define FREQ_ONE_MHZ     (1000*1000)

#define MIN_LP_FREQ_MHZ (0)
#define MAX_LP_FREQ_MHZ (2150)

#define MIN_BYPASS_FREQ_MHZ (2150)
#define MAX_BYPASS_FREQ_MHZ (2750)

#define MIN_HP_FREQ_MHZ (2750)
#define MID1_HP_FREQ_MHZ (3600)
#define MID2_HP_FREQ_MHZ (5100)
#define MAX_HP_FREQ_MHZ (7250)

#define MIN_LO_FREQ_HZ (84375000)
#define MAX_LO_FREQ_HZ (5400000000ULL)

static uint32_t max2837_freq_nominal_hz=2560000000;

uint64_t freq_cache = 100000000;
/*
 * Set freq/tuning between 0MHz to 7250 MHz (less than 16bits really used)
 * hz between 0 to 999999 Hz (not checked)
 * return false on error or true if success.
 */
bool set_freq(const uint64_t freq)
{
	bool success;
	uint32_t RFFC5071_freq_mhz;
	uint32_t MAX2837_freq_hz;
	uint64_t real_RFFC5071_freq_hz;

	const uint32_t freq_mhz = freq / 1000000;
	const uint32_t freq_hz = freq % 1000000;

	success = true;

	const max2837_mode_t prior_max2837_mode = max2837_mode(&max2837);
	max2837_set_mode(&max2837, MAX2837_MODE_STANDBY);
	if(freq_mhz < MAX_LP_FREQ_MHZ)
	{
		rf_path_set_filter(&rf_path, RF_PATH_FILTER_LOW_PASS);
		/* IF is graduated from 2650 MHz to 2343 MHz */
		max2837_freq_nominal_hz = 2650000000 - (freq / 7);
		RFFC5071_freq_mhz = (max2837_freq_nominal_hz / FREQ_ONE_MHZ) + freq_mhz;
		/* Set Freq and read real freq */
		real_RFFC5071_freq_hz = rffc5071_set_frequency(&rffc5072, RFFC5071_freq_mhz);
		max2837_set_frequency(&max2837, real_RFFC5071_freq_hz - freq);
		sgpio_cpld_stream_rx_set_q_invert(&sgpio_config, 1);
	}else if( (freq_mhz >= MIN_BYPASS_FREQ_MHZ) && (freq_mhz < MAX_BYPASS_FREQ_MHZ) )
	{
		rf_path_set_filter(&rf_path, RF_PATH_FILTER_BYPASS);
		MAX2837_freq_hz = (freq_mhz * FREQ_ONE_MHZ) + freq_hz;
		/* RFFC5071_freq_mhz <= not used in Bypass mode */
		max2837_set_frequency(&max2837, MAX2837_freq_hz);
		sgpio_cpld_stream_rx_set_q_invert(&sgpio_config, 0);
	}else if(  (freq_mhz >= MIN_HP_FREQ_MHZ) && (freq_mhz <= MAX_HP_FREQ_MHZ) )
	{
		if (freq_mhz < MID1_HP_FREQ_MHZ) {
			/* IF is graduated from 2150 MHz to 2750 MHz */
			max2837_freq_nominal_hz = 2150000000 + (((freq - 2750000000) * 60) / 85);
		} else if (freq_mhz < MID2_HP_FREQ_MHZ) {
			/* IF is graduated from 2350 MHz to 2650 MHz */
			max2837_freq_nominal_hz = 2350000000 + ((freq - 3600000000) / 5);
		} else {
			/* IF is graduated from 2500 MHz to 2738 MHz */
			max2837_freq_nominal_hz = 2500000000 + ((freq - 5100000000) / 9);
		}
		rf_path_set_filter(&rf_path, RF_PATH_FILTER_HIGH_PASS);
		RFFC5071_freq_mhz = freq_mhz - (max2837_freq_nominal_hz / FREQ_ONE_MHZ);
		/* Set Freq and read real freq */
		real_RFFC5071_freq_hz = rffc5071_set_frequency(&rffc5072, RFFC5071_freq_mhz);
		max2837_set_frequency(&max2837, freq - real_RFFC5071_freq_hz);
		sgpio_cpld_stream_rx_set_q_invert(&sgpio_config, 0);
	}else
	{
		/* Error freq_mhz too high */
		success = false;
	}
	max2837_set_mode(&max2837, prior_max2837_mode);
	if( success ) {
		freq_cache = freq;
	}
	return success;
}

// extracted from hackRF firmware/common/rffc5071.c
// Used to set RFFC5071 upconverter to multiples of 1 MHz
// for future use in determining exact tuning frequency

#define LO_MAX 5400.0
#define REF_FREQ 50.0
#define FREQ_ONE_MHZ (1000.0*1000.0)

double  rffc5071_freq(uint16_t lo) {
	uint8_t lodiv;
	uint16_t fvco;
	uint8_t fbkdiv;
	
	/* Calculate n_lo */
	uint8_t n_lo = 0;
	uint16_t x = LO_MAX / lo;
	while ((x > 1) && (n_lo < 5)) {
		n_lo++;
		x >>= 1;
	}

	lodiv = 1 << n_lo;
	fvco = lodiv * lo;

	if (fvco > 3200) {
		fbkdiv = 4;
	} else {
		fbkdiv = 2;
	}

	uint64_t tmp_n = ((uint64_t)fvco << 29ULL) / (fbkdiv*REF_FREQ) ;

	return (REF_FREQ * (tmp_n >> 5ULL) * fbkdiv * FREQ_ONE_MHZ)
			/ (lodiv * (1 << 24ULL));
}
uint32_t max2837_freq(uint32_t freq){
	uint32_t div_frac;
	//	uint32_t div_int;
	uint32_t div_rem;
	uint32_t div_cmp;
	int i;

	/* ASSUME 40MHz PLL. Ratio = F*(4/3)/40,000,000 = F/30,000,000 */
	//	div_int = freq / 30000000;
       	div_rem = freq % 30000000;
	div_frac = 0;
	div_cmp = 30000000;
	for( i = 0; i < 20; i++) {
		div_frac <<= 1;
		div_cmp >>= 1;
		if (div_rem > div_cmp) {
			div_frac |= 0x1;
			div_rem -= div_cmp;
		}
	}
	return div_rem;

}





#endif

void closedown(int a){
  errmsg("caught signal %d: %s\n",a,strsignal(a));
  hackrf_close(HackCD.device);
  hackrf_exit();

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


