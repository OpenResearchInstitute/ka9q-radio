// $Id: hackrf.c,v 1.1 2018/06/23 01:50:33 karn Exp karn $
// Read from HackRF
// Multicast raw 8-bit I/Q samples
// Accept control commands from UDP socket
#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <locale.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <libhackrf/hackrf.h>


#include "sdr.h"
#include "radio.h"
#include "misc.h"
#include "multicast.h"
#include "decimate.h"


// decibel limits for power
float Upper_threshold = 30;
float Lower_threshold = 10;
float DB_offset = 90.31;
int Agc_holdoff = 10;
#define BUFFERSIZE (1<<20)
int ADC_samprate = 3072000;
int Decimate = 16;
int Blocksize = 350;
int Device = 0;
float const DC_alpha = 0.00001;    // high pass filter coefficient for DC offset estimates, per sample
float const Power_alpha = 0.001; // high pass filter coefficient for power and I/Q imbalance estimates, per sample


int Verbose;
int Device;       // Which of several to use
int Offset;       // Offset tuner by +Fs/4, downconvert in software to avoid DC

struct sdrstate {
  // Stuff for sending commands
  struct status status;     // Frequency and gain settings, grouped for transmission in RTP packet

  // Analog gain settings
  int agc_holdoff_count;

  float power;              // Running estimate of unfiltered signal power
  long long clips;

  hackrf_device *device;
  float DC_i;
  float DC_q;
  float sinphi;          // smoothed estimate of I/Q phase error
  float imbalance;       // Ratio of I power to Q power

};

struct sdrstate HackCD;
char *Locale;


pthread_t Display_thread;
pthread_t Process_thread;


int Rtp_sock; // Socket handle for sending real time stream *and* receiving commands
int Ctl_sock;
extern int Mcast_ttl;
long Ssrc;
int Seq = 0;
int Timestamp = 0;

char Sampbuffer[BUFFERSIZE];
int Samp_wp;
int Samp_rp;


pthread_mutex_t Buf_mutex;
pthread_cond_t Buf_cond;

void *display(void *arg);
double  rffc5071_freq(uint16_t lo);
uint32_t max2837_freq(uint32_t freq);


// Callback called with incoming receiver data from A/D
int rx_callback(hackrf_transfer *transfer){
  int remain = transfer->valid_length;
  unsigned char *dp = transfer->buffer;

  while(remain > 0){
    int chunk = min(remain,BUFFERSIZE-Samp_wp);
    memcpy(Sampbuffer + Samp_wp,dp,chunk);
    dp += chunk;
    remain -= chunk;
    pthread_mutex_lock(&Buf_mutex);
    Samp_wp += chunk;
    Samp_wp &= (BUFFERSIZE-1);
    pthread_cond_signal(&Buf_cond);
    pthread_mutex_unlock(&Buf_mutex);
  }
  return 0;
}

void *process(void *arg){

  unsigned char buffer[200+2*Blocksize*sizeof(short)];
  struct rtp_header rtp;
  memset(&rtp,0,sizeof(rtp));
  rtp.version = RTP_VERS;
  rtp.type = IQ_PT;
  rtp.ssrc = Ssrc;
  int rotate_phase = 0;

  // Gain and phase corrections. These will be updated every block
  float gain_q = 1;
  float gain_i = 1;
  float secphi = 1;
  float tanphi = 0;

  
  while(1){

    rtp.timestamp = Timestamp;
    rtp.seq = Seq++;

    unsigned char *dp = buffer;
    dp = hton_rtp(dp,&rtp);
    dp = hton_status(dp,&HackCD.status);
    signed short *up = (signed short *)dp;

    pthread_mutex_lock(&Buf_mutex);
    while(1){
      int avail = Samp_wp - Samp_rp;
      if(avail < 0)
	avail += BUFFERSIZE;
      if(avail >= 2*Blocksize*Decimate)
	break;
      pthread_cond_wait(&Buf_cond,&Buf_mutex);
    }
    pthread_mutex_unlock(&Buf_mutex);
    

    float samp_i_sum = 0, samp_q_sum = 0;        // sums of I and Q, for DC offset
    float samp_i_sq_sum = 0, samp_q_sq_sum = 0;  // sums of I^2 and Q^2, for power and gain balance
    float dotprod = 0;                           // sum of I*Q, for phase balance

    for(int i=0;i<Blocksize;i++){
      complex float samp;
      for(int j=0; j < Decimate; j++){
	float samp_i = Sampbuffer[Samp_rp++];
	Samp_rp &= (BUFFERSIZE-1);
	float samp_q = Sampbuffer[Samp_rp++];
	Samp_rp &= (BUFFERSIZE-1);
	if(fabsf(samp_q) >= 127 || fabsf(samp_i) >= 127)
	  HackCD.clips++;
	
	// Remove and update DC offsets
	// Is this necessary when Offset is active?
	samp_i_sum += samp_i;
	samp_q_sum += samp_q;
	samp_i -= HackCD.DC_i;
	samp_q -= HackCD.DC_q;
	
	// Must correct gain and phase before frequency shift
	// accumulate I and Q energies before gain correction
	samp_i_sq_sum += samp_i * samp_i;
	samp_q_sq_sum += samp_q * samp_q;
	
	// Balance gains, keeping constant total energy
	samp_i *= gain_i;                  samp_q *= gain_q;
	
	// Accumulate phase error
	dotprod += samp_i * samp_q;
	
	// Correct phase
	samp_q = secphi * samp_q - tanphi * samp_i;
	

	if(Offset){
	  // Increase frequency by Fs/4 to compensate for tuner being high by Fs/4
	  switch(rotate_phase){
	  default:
	  case 0:
	    samp = decimate(NULL,samp_i,samp_q);
	    break; // leave unchanged
	  case 1:
	    samp = decimate(NULL,-samp_q,samp_i);
	    break;
	  case 2:
	    samp = decimate(NULL,-samp_i,-samp_q);
	    break;
	  case 3:
	    samp = decimate(NULL,samp_q,-samp_i);
	    break;
	  }
	  rotate_phase++;
	  rotate_phase &= 3; // Modulo 4
	} else {
	  samp = decimate(NULL,samp_i,samp_q);
	}

      }
      *up++ = (short)crealf(samp);
      *up++ = (short)cimagf(samp);
    }
    // Update every block of Blocksize*Decimate input sample pairs
    // estimates of DC offset, signal powers and phase error
    HackCD.DC_i += DC_alpha * (samp_i_sum/(Blocksize*Decimate) - HackCD.DC_i);
    HackCD.DC_q += DC_alpha * (samp_q_sum/(Blocksize*Decimate) - HackCD.DC_q);
    HackCD.imbalance += Power_alpha * ( (samp_i_sq_sum / samp_q_sq_sum) - HackCD.imbalance);
    float block_energy = samp_i_sq_sum + samp_q_sq_sum;
    HackCD.power += Power_alpha * (block_energy/(2*Blocksize*Decimate) - HackCD.power); // Average A/D output power per channel
    
    float dpn = 2 * dotprod / block_energy;
    HackCD.sinphi += Power_alpha * (dpn - HackCD.sinphi);
    gain_q = sqrtf(0.5 * (1 + HackCD.imbalance));
    gain_i = sqrtf(0.5 * (1 + 1./HackCD.imbalance));
    secphi = 1/sqrtf(1 - HackCD.sinphi * HackCD.sinphi); // sec(phi) = 1/cos(phi)
    tanphi = HackCD.sinphi * secphi;                     // tan(phi) = sin(phi) * sec(phi) = sin(phi)/cos(phi)
    
    dp = (unsigned char *)up;
    if(send(Rtp_sock,buffer,dp - buffer,0) == -1){
      perror("send");
      // If we're sending to a unicast address without a listener, we'll get ECONNREFUSED
      // Sleep 1 sec to slow down the rate of these messages
      usleep(1000000);
    }
    Timestamp += Blocksize; // samples
  
    // Simply increment by number of samples
    // But what if we lose some? Then the clock will always be off
    HackCD.status.timestamp += 1.e9 * (Blocksize) / (ADC_samprate / Decimate);
  }
}


int main(int argc,char *argv[]){
  // if we have root, up our priority and drop privileges
  int prio = getpriority(PRIO_PROCESS,0);
  prio = setpriority(PRIO_PROCESS,0,prio - 10);

  // Quickly drop root if we have it
  // The sooner we do this, the fewer options there are for abuse
  if(seteuid(getuid()) != 0)
    perror("seteuid");

  char *dest = "239.1.6.1"; // Default for testing

  Locale = getenv("LANG");
  if(Locale == NULL || strlen(Locale) == 0)
    Locale = "en_US.UTF-8";

  int c;
  while((c = getopt(argc,argv,"d:vp:l:b:R:T:o")) != EOF){
    switch(c){
    case 'o':
      Offset++;
      break;
    case 'R':
      dest = optarg;
      break;
    case 'd':
      Device = strtol(optarg,NULL,0);
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
    fprintf(stderr,"hackrf device %d: blocksize %d\n",Device,Blocksize);

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
  if(getsockname(Rtp_sock,(struct sockaddr *)&ctl_sockaddr,&siz) == -1){
    perror("getsockname on ctl port");
    exit(1);
  }
  struct sockaddr_in locsock;
  locsock.sin_family = AF_INET;
  locsock.sin_port = htons(ntohs(ctl_sockaddr.sin_port)+1);
  locsock.sin_addr.s_addr = INADDR_ANY;
  bind(Ctl_sock,(struct sockaddr *)&locsock,sizeof(locsock));

  int ret;
  if((ret = hackrf_init()) != HACKRF_SUCCESS){
    fprintf(stderr,"hackrf_init() failed: %s\n",hackrf_error_name(ret));
    exit(1);
  }
  // Enumerate devices
  hackrf_device_list_t *dlist = hackrf_device_list();

  if((ret = hackrf_device_list_open(dlist,Device,&HackCD.device)) != HACKRF_SUCCESS){
    fprintf(stderr,"hackrf_open(%d) failed: %s\n",Device,hackrf_error_name(ret));
    exit(1);
  }
  hackrf_device_list_free(dlist); dlist = NULL;

  hackrf_set_sample_rate(HackCD.device,(double)ADC_samprate);
  HackCD.status.samprate = ADC_samprate / Decimate;
  uint32_t bw = hackrf_compute_baseband_filter_bw_round_down_lt(ADC_samprate);
  hackrf_set_baseband_filter_bandwidth(HackCD.device,bw);

  // NOTE: what we call mixer gain, they call lna gain
  // What we call lna gain, they call antenna enable

  HackCD.status.lna_gain = 1;
  HackCD.status.mixer_gain = 20;
  HackCD.status.if_gain = 20;

  hackrf_set_antenna_enable(HackCD.device,HackCD.status.lna_gain);
  hackrf_set_lna_gain(HackCD.device,HackCD.status.mixer_gain); // 
  hackrf_set_vga_gain(HackCD.device,HackCD.status.if_gain);

  uint64_t intfreq = HackCD.status.frequency = 146000000;
  if(Offset)
    intfreq += ADC_samprate / 4; // Offset tune high by +Fs/4

  hackrf_set_freq(HackCD.device,intfreq);

  pthread_mutex_init(&Buf_mutex,NULL);
  pthread_cond_init(&Buf_cond,NULL);

  decimate_setup(3.0);

  pthread_create(&Process_thread,NULL,process,NULL);

  hackrf_start_rx(HackCD.device,rx_callback,&HackCD);

  signal(SIGPIPE,SIG_IGN);
  signal(SIGINT,closedown);
  signal(SIGKILL,closedown);
  signal(SIGQUIT,closedown);
  signal(SIGTERM,closedown);        

  if(Verbose > 1)
    pthread_create(&Display_thread,NULL,display,NULL);

  time_t tt;
  time(&tt);

  Ssrc = tt & 0xffffffff; // low 32 bits of clock time

  // Process commands to change hackrf state
  // We listen on the same IP address and port we use as a multicasting source
  pthread_setname("hackrf-cmd");

  while(1){

    fd_set fdset;
    socklen_t addrlen;
    int r;
    struct timeval timeout;
    struct status requested_status;
    
    // Read with a timeout - necessary?
    FD_ZERO(&fdset);
    FD_SET(Ctl_sock,&fdset);
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    r = select(Ctl_sock+1,&fdset,NULL,NULL,&timeout);
    if(r == -1){
      perror("select");
      usleep(50000); // don't loop tightly
      continue;
    }
    if(r == 0)
      continue;

    // A command arrived, read it
    // Should probably log these
    struct sockaddr_in6 command_address;
    addrlen = sizeof(command_address);
    if((r = recvfrom(Ctl_sock,&requested_status,sizeof(requested_status),0,(struct sockaddr *)&command_address,&addrlen)) <= 0){
      if(r < 0)
	perror("recv");
      usleep(50000); // don't loop tightly
      continue;
    }
    
    if(r < sizeof(requested_status))
      continue; // Too short; ignore
    
    uint64_t intfreq = HackCD.status.frequency = requested_status.frequency;
    if(Offset)
      intfreq += ADC_samprate/4; // Offset tune by +Fs/4

    hackrf_set_freq(HackCD.device,intfreq);
    struct timeval tp;
    gettimeofday(&tp,NULL);
    // Timestamp is in nanoseconds for futureproofing, but time of day is only available in microsec
    HackCD.status.timestamp = ((tp.tv_sec - UNIX_EPOCH + GPS_UTC_OFFSET) * 1000000LL + tp.tv_usec) * 1000LL;
  }
  // Can't really get here
  close(Rtp_sock);
  hackrf_close(HackCD.device);
  hackrf_exit();
  exit(0);
}



// Status display thread
void *display(void *arg){
  pthread_setname("hackrf-disp");

  double  rffc5071_freq(uint16_t lo);
  uint32_t max2837_freq(uint32_t freq);

  fprintf(stderr,"               |-------GAINS-----| Level DC offsets  ---Errors--          clips\n");
  fprintf(stderr,"Frequency      LNA  mixer baseband  A/D     I     Q  phase  gain\n");
  fprintf(stderr,"Hz                     dB  dB        dB                deg    dB\n");   

  while(1){
    float powerdB = 10*log10f(HackCD.power);

    fprintf(stderr,"%'-15.0lf%3d%7d%4d%'10.1f%6.1f%6.1f%7.2f%6.2f%'16lld\r",
	    HackCD.status.frequency,
	    HackCD.status.lna_gain,	    
	    HackCD.status.mixer_gain,
	    HackCD.status.if_gain,
	    powerdB,
	    HackCD.DC_i,
	    HackCD.DC_q,
	    (180/M_PI) * asin(HackCD.sinphi),
	    10*log10(HackCD.imbalance),
	    HackCD.clips);
    usleep(100000); // 10 Hz
    if(HackCD.agc_holdoff_count){
      HackCD.agc_holdoff_count--;
    } else if(isnan(powerdB) || powerdB < Lower_threshold){
      // Turn on external antenna first, start counter
      if(HackCD.status.lna_gain < 1){
	HackCD.status.lna_gain++;
	HackCD.agc_holdoff_count = Agc_holdoff;
	hackrf_set_antenna_enable(HackCD.device,HackCD.status.lna_gain);
      } else if(HackCD.status.mixer_gain < 40){
	HackCD.status.mixer_gain += 8;
	hackrf_set_lna_gain(HackCD.device,HackCD.status.mixer_gain);
	HackCD.agc_holdoff_count = Agc_holdoff;
      } else if(HackCD.status.if_gain < 62){
	HackCD.status.if_gain += 2;
	hackrf_set_vga_gain(HackCD.device,HackCD.status.if_gain);
	HackCD.agc_holdoff_count = Agc_holdoff;
      }
    } else if(powerdB > Upper_threshold){
      // Reduce gain (IF first), start counter
      if(HackCD.status.if_gain > 0){
	HackCD.status.if_gain -= 2;
	hackrf_set_vga_gain(HackCD.device,HackCD.status.if_gain);
	HackCD.agc_holdoff_count = Agc_holdoff;
      } else if(HackCD.status.mixer_gain > 0){
	HackCD.status.mixer_gain -= 8;
	hackrf_set_lna_gain(HackCD.device,HackCD.status.mixer_gain);
	HackCD.agc_holdoff_count = Agc_holdoff;
      } else if(HackCD.status.lna_gain > 0){
	HackCD.status.lna_gain--;
	hackrf_set_antenna_enable(HackCD.device,HackCD.status.lna_gain);
      }
    }
  }
  return NULL;
}



// If we don't stop the A/D, it'll take several seconds to overflow and stop by itself,
// and during that time we can't restart
void closedown(int a){
  if(Verbose)
    fprintf(stderr,"\nhackrf: caught signal %d: %s\n",a,strsignal(a));

  exit(1);
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
#if 0

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

#endif
