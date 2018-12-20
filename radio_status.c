#define _GNU_SOURCE 1
#include <assert.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <pthread.h>
#if defined(linux)
#include <bsd/string.h>
#endif
#include <math.h>
#include <complex.h>
#undef I
#include <sys/time.h>
#include <ncurses.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netdb.h>

#include "misc.h"
#include "dsp.h"
#include "radio.h"
#include "filter.h"
#include "multicast.h"
#include "status.h"


void send_radio_status(struct demod *demod,int full);
void decode_radio_commands(struct demod *, unsigned char *, int);

struct state State[256];

// Thread to periodically transmit receiver state
void *send_status(void *arg){
  pthread_setname("status");
  assert(arg != NULL);
  struct demod * const demod = arg;

  memset(State,0,sizeof(State));
  
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000; // 100 ms

  if(setsockopt(demod->output.ctl_fd,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv))){
    perror("ncmd setsockopt");
    return NULL;
  }
  int counter = 0;
  while(1){
    unsigned char buffer[8192];
    memset(buffer,0,sizeof(buffer));
    int length = recv(demod->output.ctl_fd,buffer,sizeof(buffer),0); // Waits up to 100 ms for command
    if(length > 0){
      // Parse entries
      unsigned char *cp = buffer;

      int cr = *cp++; // Command/response
      if(cr == 0)
	continue; // Ignore our own status messages
      demod->output.commands++;
      decode_radio_commands(demod,cp,length-1);
      counter = 0; // Send complete status in response
    }
    send_radio_status(demod,(counter == 0));
    if(counter-- <= 0)
      counter = 10;
  }
}

void send_radio_status(struct demod *demod,int full){
  unsigned char packet[2048],*bp;
  memset(packet,0,sizeof(packet));
  bp = packet;
  
  *bp++ = 0; // Response (not a command);
  
  encode_int(&bp,COMMAND_TAG,demod->output.command_tag);
  encode_int64(&bp,COMMANDS,demod->output.commands); // integer

  if(strlen(demod->input.description) > 0)
    encode_string(&bp,DESCRIPTION,demod->input.description,strlen(demod->input.description));

  // Echo timestamp from source
  encode_int64(&bp,GPS_TIME,demod->sdr.status.timestamp); // integer
  // Who's sending us I/Q data
  encode_socket(&bp,INPUT_DATA_SOURCE_SOCKET,&demod->input.data_source_address);
  // Destination address for I/Q data
  encode_socket(&bp,INPUT_DATA_DEST_SOCKET,&demod->input.data_dest_address);
  // Source of metadata
  encode_socket(&bp,INPUT_METADATA_SOURCE_SOCKET,&demod->input.metadata_source_address);
  // Destination address (usually multicast) and port on which we're getting metadata
  encode_socket(&bp,INPUT_METADATA_DEST_SOCKET,&demod->input.metadata_dest_address);
  encode_int32(&bp,INPUT_SSRC,demod->input.rtp.ssrc);
  encode_int32(&bp,INPUT_SAMPRATE,demod->input.samprate); // integer
  encode_int64(&bp,INPUT_METADATA_PACKETS,demod->input.metadata_packets); // integer
  encode_int64(&bp,INPUT_DATA_PACKETS,demod->input.rtp.packets);
  encode_int64(&bp,INPUT_SAMPLES,demod->input.samples);
  encode_int64(&bp,INPUT_DROPS,demod->input.rtp.drops);
  encode_int64(&bp,INPUT_DUPES,demod->input.rtp.dupes);

  // Source address we're using to send data
  encode_socket(&bp,OUTPUT_DATA_SOURCE_SOCKET,&demod->output.data_source_address);
  // Where we're sending PCM output
  encode_socket(&bp,OUTPUT_DATA_DEST_SOCKET,&demod->output.data_dest_address);

  encode_int32(&bp,OUTPUT_SSRC,demod->output.rtp.ssrc);
  encode_byte(&bp,OUTPUT_TTL,Mcast_ttl);
  encode_int32(&bp,OUTPUT_SAMPRATE,demod->output.samprate);
  encode_int64(&bp,OUTPUT_DATA_PACKETS,demod->output.rtp.packets);
  encode_int64(&bp,OUTPUT_METADATA_PACKETS,demod->output.metadata_packets);
  
  // Tuning
  encode_double(&bp,RADIO_FREQUENCY,get_freq(demod)); // Hz
  encode_double(&bp,SECOND_LO_FREQUENCY,get_second_LO(demod)); // Hz
  encode_double(&bp,SHIFT_FREQUENCY,demod->tune.shift); // Hz
  
  // Front end - passed through from SDR metadata
  encode_double(&bp,AD_LEVEL,demod->sdr.ad_level);
  encode_double(&bp,FIRST_LO_FREQUENCY,demod->sdr.status.frequency); // Hz
  encode_byte(&bp,LNA_GAIN,demod->sdr.status.lna_gain); // dB
  encode_byte(&bp,MIXER_GAIN,demod->sdr.status.mixer_gain); // dB
  encode_byte(&bp,IF_GAIN,demod->sdr.status.if_gain); // dB
  encode_float(&bp,DC_I_OFFSET,demod->sdr.DC_i); // relative 1
  encode_float(&bp,DC_Q_OFFSET,demod->sdr.DC_q); // relative 1
  encode_float(&bp,IQ_IMBALANCE,demod->sdr.imbalance); // dB
  encode_float(&bp,IQ_PHASE,demod->sdr.sinphi); // sine - dimensionless
  encode_double(&bp,CALIBRATE,demod->sdr.calibration); // dimensionless
  
  // Doppler info
  encode_double(&bp,DOPPLER_FREQUENCY,get_doppler(demod));
  encode_double(&bp,DOPPLER_FREQUENCY_RATE,get_doppler_rate(demod));
  
  // Filtering
  encode_float(&bp,LOW_EDGE,demod->filter.low);
  encode_float(&bp,HIGH_EDGE,demod->filter.high);
  encode_float(&bp,KAISER_BETA,demod->filter.kaiser_beta);
  encode_int32(&bp,FILTER_BLOCKSIZE,demod->filter.L);
  encode_int32(&bp,FILTER_FIR_LENGTH,demod->filter.M);
  if(demod->filter.out)
    encode_float(&bp,NOISE_BANDWIDTH,demod->input.samprate * demod->filter.out->noise_gain); // Hz
  
  // Signals - these ALWAYS change
  encode_float(&bp,IF_POWER,power2dB(demod->sig.if_power));
  encode_float(&bp,BASEBAND_POWER,power2dB(demod->sig.bb_power));
  encode_float(&bp,NOISE_DENSITY,power2dB(demod->sig.n0));
  
  // Demodulation mode
  encode_byte(&bp,DEMOD_TYPE,demod->demod_type);
  encode_int32(&bp,OUTPUT_CHANNELS,demod->output.channels);
  switch(demod->demod_type){
  case FM_DEMOD:
    encode_byte(&bp,FM_FLAT,demod->opt.flat);
    encode_float(&bp,DEMOD_SNR,power2dB(demod->sig.snr));
    encode_float(&bp,FREQ_OFFSET,demod->sig.foffset);
    encode_float(&bp,PEAK_DEVIATION,demod->sig.pdeviation);
    encode_float(&bp,PL_TONE,demod->sig.plfreq);
    break;
  case LINEAR_DEMOD:
    encode_byte(&bp,INDEPENDENT_SIDEBAND,demod->filter.isb);
    encode_byte(&bp,PLL_ENABLE,demod->opt.pll);
    if(demod->opt.pll){
      encode_byte(&bp,PLL_LOCK,demod->sig.pll_lock);
      encode_byte(&bp,PLL_SQUARE,demod->opt.square);
      encode_float(&bp,PLL_PHASE,demod->sig.cphase); // radians
      encode_byte(&bp,ENVELOPE,demod->opt.env);
      encode_float(&bp,DEMOD_SNR,power2dB(demod->sig.snr));
      encode_float(&bp,FREQ_OFFSET,demod->sig.foffset);
    }
    encode_float(&bp,GAIN,voltage2dB(demod->agc.gain));
    encode_byte(&bp,AGC_ENABLE,demod->opt.agc);
    encode_float(&bp,HEADROOM,voltage2dB(demod->agc.headroom));
    encode_float(&bp,AGC_HANGTIME,demod->agc.hangtime / demod->output.samprate); // samples -> sec
    encode_float(&bp,AGC_RECOVERY_RATE,voltage2dB(demod->agc.recovery_rate) * demod->output.samprate);
    encode_float(&bp,AGC_ATTACK_RATE,voltage2dB(demod->agc.attack_rate) * demod->output.samprate); // amplitude/sample -> dB/s
    break;
  }
  encode_float(&bp,OUTPUT_LEVEL,power2dB(demod->output.level));

  encode_eol(&bp);
  
  int len = compact_packet(&State[0],packet,full);
  send(demod->output.status_fd,packet,len,0);
  demod->output.metadata_packets++;
}

void decode_radio_commands(struct demod *demod,unsigned char *buffer,int length){
  unsigned char *cp = buffer;
  int fset = 0;
  double nrf = NAN;
  double nlo2 = NAN;
  double nlo1 = NAN;

  while(cp - buffer < length){
    enum status_type type = *cp++; // increment cp to length field
    
    if(type == EOL)
      break; // End of list
    
    unsigned int optlen = *cp++;
    if(cp - buffer + optlen >= length)
      break; // Invalid length
    

    int i;
    float f;
    double d;

    switch(type){
    case EOL: // Shouldn't get here
      break;
    case DEMOD_TYPE:
      i = decode_int(cp,optlen);
      if(demod->demod_type != i){
	// Notify demod threads of the change
	pthread_mutex_lock(&demod->demod_mutex);
	demod->demod_type = i;
	pthread_cond_broadcast(&demod->demod_cond);
	pthread_mutex_unlock(&demod->demod_mutex);
      }
      break;
    case CALIBRATE:
      d = decode_double(cp,optlen); // dimensionless
      if(!isnan(d))
	demod->sdr.calibration = d;
      break;
    case RADIO_FREQUENCY:  // Hz
      nrf = decode_double(cp,optlen);
      break;
    case FIRST_LO_FREQUENCY:  // Hz
      nlo1 = decode_double(cp,optlen);
      break;
    case SECOND_LO_FREQUENCY:  // Hz
      nlo2 = decode_double(cp,optlen);
      break;
    case LOW_EDGE:  // Hz
      f = decode_float(cp,optlen);
      if(!isnan(f))
	demod->filter.low = f;
      fset++;
      break;
    case HIGH_EDGE: // Hz
      f = decode_float(cp,optlen);
      if(!isnan(f))
	demod->filter.high = f;
      fset++;
      break;
    case SHIFT_FREQUENCY: // Hz
      d = decode_double(cp,optlen);
      if(!isnan(d)){
	demod->tune.shift = d;
	set_shift(demod,demod->tune.shift);
      }
      break;
    case KAISER_BETA: // dimensionless
      f = decode_float(cp,optlen);
      if(!isnan(f)){
	demod->filter.kaiser_beta = f < 0 ? 0 : f;
	fset++;
      }
      break;
    case INDEPENDENT_SIDEBAND: // boolean
      demod->filter.isb = decode_int(cp,optlen);
      break;
    case PLL_ENABLE: // boolean
      demod->opt.pll = decode_int(cp,optlen);
      break;
    case PLL_SQUARE: // boolean
      demod->opt.square = decode_int(cp,optlen);
      break;
    case FM_FLAT:  // boolean
      demod->opt.flat = decode_int(cp,optlen);
      break;
    case AGC_ENABLE: // boolean
      demod->opt.agc = decode_int(cp,optlen);
      break;
    case OUTPUT_CHANNELS: // integer (1 or 2)
      demod->output.channels = decode_int(cp,optlen);
      break;
    case COMMAND_TAG:     // dimensionless, opaque integer
      demod->output.command_tag = decode_int(cp,optlen);
      break;
    case GAIN:
      f = decode_float(cp,optlen);
      if(!isnan(f))
	demod->agc.gain = powf(10.,f/20);
      break;
    case HEADROOM:        // dB -> amplitude ratio < 1
      f = decode_float(cp,optlen);
      if(!isnan(f)){
	demod->agc.headroom = powf(10.,-fabs(f/20.));
      }
      break;
    case AGC_HANGTIME:    // sec -> samples
      f = decode_float(cp,optlen);
      if(!isnan(f)){
	demod->agc.hangtime = fabsf(f) * demod->output.samprate;
      }
      break;
    case AGC_RECOVERY_RATE: // dB/sec -> amplitude ratio/sample > 1
      f = decode_float(cp,optlen);
      if(!isnan(f))
	demod->agc.recovery_rate = powf(10., fabs(f / 20. / demod->output.samprate));
      break;
    case AGC_ATTACK_RATE:   // dB/sec -> amplitude ratio/sample < 1
      f = decode_float(cp,optlen);
      if(!isnan(f))
	demod->agc.attack_rate = powf(10., -fabs(f / 20. / demod->output.samprate));
      break;
    case ENVELOPE:  // boolean
      demod->opt.env = decode_int(cp,optlen);
      break;
    default:
      break;
    }
    cp += optlen;
  }
  if(fset){
    double samptime = 1./demod->output.samprate;
    set_filter(demod->filter.out,samptime*demod->filter.low,samptime*demod->filter.high,demod->filter.kaiser_beta);
  }
  // Tuning changed?
  if(!isnan(nrf)) // Specific RF frequency always takes precedence, nlo2 used if possible
    set_freq(demod,nrf,nlo2);
  else if(!isnan(nlo2) && LO2_in_range(demod,nlo2,0)){
    // Tune around with fixed LO1
    nrf = get_freq(demod) - (nlo2 - get_second_LO(demod));
    set_freq(demod,nrf,nlo2);
  } else if(!isnan(nlo1)){
    // Will automatically change LO2 when LO1 actually changes
    set_first_LO(demod,nlo1);
  }
}


void decode_sdr_status(struct demod *demod,unsigned char *buffer,int length);

void *recv_sdr_status(void *arg){
  struct demod *demod = (struct demod *)arg;

  // Solicit immediate full status
  unsigned char packet[8192],*bp;
  memset(packet,0,sizeof(packet));
  bp = packet;
  *bp++ = 1; // Command
  encode_eol(&bp);
  int len = bp - packet;
  send(demod->input.ctl_fd,packet,len,0);

  while(1){
    unsigned char buffer[8192];

    memset(buffer,0,sizeof(buffer));
    socklen_t socklen;
    int len = recvfrom(demod->input.status_fd,buffer,sizeof(buffer),0,(struct sockaddr *)&demod->input.metadata_source_address,&socklen);
    if(len <= 0){
      sleep(1);
      continue;
    }
    // Parse entries
    int cr = buffer[0]; // command-response byte

    if(cr == 1)
      continue; // Ignore commands
    
    demod->input.metadata_packets++;
    decode_sdr_status(demod,buffer+1,len-1);
    pthread_mutex_lock(&demod->sdr.status_mutex);
    pthread_cond_broadcast(&demod->sdr.status_cond);
    pthread_mutex_unlock(&demod->sdr.status_mutex);
  }    
}

void decode_sdr_status(struct demod *demod,unsigned char *buffer,int length){
  unsigned char *cp = buffer;
  double nfreq = NAN;
  int nsamprate = 0;

  while(cp - buffer < length){
    enum status_type type = *cp++; // increment cp to length field

    if(type == EOL)
      break; // End of list

    double d;
    float f;
    
    unsigned int optlen = *cp++;
    if(cp - buffer + optlen >= length)
      break; // Invalid length
    switch(type){
    case EOL: // Shouldn't get here since it's checked above
      goto done;
    case DESCRIPTION:
      decode_string(cp,optlen,&demod->input.description,sizeof(demod->input.description));
      break;
    case OUTPUT_DATA_DEST_SOCKET:
      // SDR data destination address (usually multicast)
      // Becomes our data input socket
      decode_socket(&demod->input.data_dest_address,cp,optlen);
      break;
    case RADIO_FREQUENCY:
      nfreq = decode_double(cp,optlen);
      break;
    case INPUT_SAMPRATE:
      demod->sdr.status.samprate = decode_int(cp,optlen);
      break;
    case OUTPUT_SAMPRATE:
      nsamprate = decode_int(cp,optlen);
      if(nsamprate != demod->input.samprate){
	demod->input.samprate = nsamprate;
	demod->filter.decimate = demod->input.samprate / demod->output.samprate;
	if(demod->filter.out)
	  set_filter(demod->filter.out,
		     demod->filter.low/demod->output.samprate,
		     demod->filter.high/demod->output.samprate,
		     demod->filter.kaiser_beta);
      }
      break;
    case GPS_TIME:
      demod->sdr.status.timestamp = decode_int(cp,optlen);
      break;
    case AD_LEVEL:
      demod->sdr.ad_level = decode_float(cp,optlen);
      break;
    case LOW_EDGE:
      f = decode_float(cp,optlen);
      if(!isnan(f))
	demod->sdr.min_IF = f;
      break;
    case HIGH_EDGE:
      f = decode_float(cp,optlen);
      if(!isnan(f))
	demod->sdr.max_IF = f;
      break;
    case LNA_GAIN:
      demod->sdr.status.lna_gain = decode_int(cp,optlen);
      break;
    case MIXER_GAIN:
      demod->sdr.status.mixer_gain = decode_int(cp,optlen);
      break;
    case IF_GAIN:
      demod->sdr.status.if_gain = decode_int(cp,optlen);
      break;
    case GAIN: // Overall SDR gain (entirely analog)
      f = decode_float(cp,optlen);
      demod->sdr.gain_factor = powf(10.,-f/20); // Amplitude ratio to make overall gain unity
      break;
    case DC_I_OFFSET:
      f = decode_float(cp,optlen);
      if(!isnan(f))
	demod->sdr.DC_i = f;
      break;
    case DC_Q_OFFSET:
      f = decode_float(cp,optlen);
      if(!isnan(f))
	demod->sdr.DC_q = f;
      break;
    case IQ_IMBALANCE:
      f = decode_float(cp,optlen);
      if(!isnan(f))
	demod->sdr.imbalance = f;
      break;
    case IQ_PHASE:
      f = decode_float(cp,optlen);
      if(!isnan(f))
	demod->sdr.sinphi = f;
      break;
    case CALIBRATE:
      d = decode_float(cp,optlen);
      if(!isnan(d))
	demod->sdr.calibration = d;
      break;
    case DIRECT_CONVERSION:
      demod->sdr.direct_conversion = decode_int(cp,optlen);
      break;
    case COMMANDS:
      demod->input.commands = decode_int(cp,optlen);
      break;
    default:
      break;
    }
    cp += optlen;
  }
  if(!isnan(nfreq) && demod->sdr.status.frequency != nfreq && demod->input.samprate != 0){
    // Recalculate LO2
    demod->sdr.status.frequency = nfreq;
    double new_LO2 = -(demod->tune.freq - get_first_LO(demod));
    set_second_LO(demod,new_LO2);
  }
  done:;
}



  
