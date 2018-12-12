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


uint64_t Commands;
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
      Commands++;
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
  encode_int64(&bp,COMMANDS,Commands);

  // Echo timestamp from source
  encode_int64(&bp,GPS_TIME,demod->sdr.status.timestamp);

  // Who's sending us I/Q data
  {
    struct sockaddr_in *sin;
    struct sockaddr_in6 *sin6;
    
    switch(demod->input.data_source_address.ss_family){
    case AF_INET:
      *bp++ = INPUT_DATA_SOURCE_SOCKET;
      sin = (struct sockaddr_in *)&demod->input.data_source_address;
      *bp++= 6;
      memcpy(bp,&sin->sin_addr.s_addr,4); // Already in network order
      bp += 4;
      memcpy(bp,&sin->sin_port,2);
      bp += 2;
      break;
    case AF_INET6:
      *bp++ = INPUT_DATA_SOURCE_SOCKET;
      sin6 = (struct sockaddr_in6 *)&demod->input.data_source_address;
      *bp++ = 10;
      memcpy(bp,&sin6->sin6_addr,8);
      bp += 8;
      memcpy(bp,&sin6->sin6_port,2);
      bp += 2;
      break;
    default:
      break;
    }
  }
  // Destination address for I/Q data
  {
    struct sockaddr_in *sin;
    struct sockaddr_in6 *sin6;

    switch(demod->input.data_dest_address.ss_family){
    case AF_INET:
      *bp++ = INPUT_DATA_DEST_SOCKET;
      sin = (struct sockaddr_in *)&demod->input.data_dest_address;
      *bp++ = 6;
      memcpy(bp,&sin->sin_addr.s_addr,4); // Already in network order
      bp += 4;
      memcpy(bp,&sin->sin_port,2);
      bp += 2;
      break;
    case AF_INET6:
      *bp++ = INPUT_DATA_DEST_SOCKET;
      sin6 = (struct sockaddr_in6 *)&demod->input.data_dest_address;
      *bp++ = 10;
      memcpy(bp,&sin6->sin6_addr,8);
      bp += 8;
      memcpy(bp,&sin6->sin6_port,2);
      bp += 2;
      break;
    default:
      break;
    }
  }
  // Source of metadata
  {
    struct sockaddr_in *sin;
    struct sockaddr_in6 *sin6;

    switch(demod->input.metadata_source_address.ss_family){
    case AF_INET:
      *bp++ = INPUT_METADATA_SOURCE_SOCKET;
      sin = (struct sockaddr_in *)&demod->input.metadata_source_address;
      *bp++= 6;
      memcpy(bp,&sin->sin_addr.s_addr,4); // Already in network order
      bp += 4;
      memcpy(bp,&sin->sin_port,2);
      bp += 2;
      break;
    case AF_INET6:
      *bp++ = INPUT_METADATA_SOURCE_SOCKET;
      sin6 = (struct sockaddr_in6 *)&demod->input.metadata_source_address;
      *bp++ = 10;
      memcpy(bp,&sin6->sin6_addr,8);
      bp += 8;
      memcpy(bp,&sin6->sin6_port,2);
      bp += 2;
      break;
    default:
      break;
    }
  }
  // Destination address (usually multicast) and port on which we're getting metadata
  {
    struct sockaddr_in *sin;
    struct sockaddr_in6 *sin6;

    switch(demod->input.metadata_dest_address.ss_family){
    case AF_INET:
      *bp++ = INPUT_METADATA_DEST_SOCKET;
      sin = (struct sockaddr_in *)&demod->input.metadata_dest_address;
      *bp++ = 6;
      memcpy(bp,&sin->sin_addr.s_addr,4); // Already in network order
      bp += 4;
      memcpy(bp,&sin->sin_port,2);
      bp += 2;
      break;
    case AF_INET6:
      *bp++ = INPUT_METADATA_DEST_SOCKET;
      sin6 = (struct sockaddr_in6 *)&demod->input.metadata_dest_address;
      *bp++ = 10;
      memcpy(bp,&sin6->sin6_addr,8);
      bp += 8;
      memcpy(bp,&sin6->sin6_port,2);
      bp += 2;
      break;
    default:
      break;
    }
  }
  encode_int32(&bp,INPUT_SSRC,demod->input.rtp.ssrc);
  encode_int32(&bp,INPUT_SAMPRATE,demod->sdr.status.samprate);
  encode_int64(&bp,INPUT_METADATA_PACKETS,demod->input.metadata_packets);
  encode_int64(&bp,INPUT_DATA_PACKETS,demod->input.rtp.packets);
  encode_int64(&bp,INPUT_SAMPLES,demod->input.samples);
  encode_int64(&bp,INPUT_DROPS,demod->input.rtp.drops);
  encode_int64(&bp,INPUT_DUPES,demod->input.rtp.dupes);

  // Source address we're using to send data
  {
    struct sockaddr_in *sin;
    struct sockaddr_in6 *sin6;
    
    switch(demod->output.data_source_address.ss_family){
    case AF_INET:
      *bp++ = OUTPUT_DATA_SOURCE_SOCKET;
      sin = (struct sockaddr_in *)&demod->output.data_source_address;
      *bp++ = 6;
      memcpy(bp,&sin->sin_addr.s_addr,4); // Already in network order
      bp += 4;
      memcpy(bp,&sin->sin_port,2);
      bp += 2;
      break;
    case AF_INET6:
      *bp++ = OUTPUT_DATA_SOURCE_SOCKET;
      sin6 = (struct sockaddr_in6 *)&demod->output.data_source_address;
      *bp++ = 10;
      memcpy(bp,&sin6->sin6_addr,8);
      bp += 8;
      memcpy(bp,&sin6->sin6_port,2);
      bp += 2;
      break;
    default:
      break;
    }
  }
  // Where we're sending PCM output
  {
    struct sockaddr_in *sin;
    struct sockaddr_in6 *sin6;

    switch(demod->output.data_dest_address.ss_family){
    case AF_INET:
      *bp++ = OUTPUT_DATA_DEST_SOCKET;
      sin = (struct sockaddr_in *)&demod->output.data_dest_address;
      *bp++ = 6;
      memcpy(bp,&sin->sin_addr.s_addr,4); // Already in network order
      bp += 4;
      memcpy(bp,&sin->sin_port,2);
      bp += 2;
      break;
    case AF_INET6:
      *bp++ = OUTPUT_DATA_DEST_SOCKET;
      sin6 = (struct sockaddr_in6 *)&demod->output.data_dest_address;
      *bp++ = 10;
      memcpy(bp,&sin6->sin6_addr,8);
      bp += 8;
      memcpy(bp,&sin6->sin6_port,2);
      bp += 2;
      break;
    default:
      break;
    }
  }

  encode_int32(&bp,OUTPUT_SSRC,demod->output.rtp.ssrc);
  encode_byte(&bp,OUTPUT_TTL,Mcast_ttl);
  encode_int32(&bp,OUTPUT_SAMPRATE,demod->output.samprate);
  encode_int64(&bp,OUTPUT_DATA_PACKETS,demod->output.rtp.packets);
  encode_int64(&bp,OUTPUT_METADATA_PACKETS,demod->output.metadata_packets);
  
  // Tuning
  encode_double(&bp,RADIO_FREQUENCY,get_freq(demod));
  encode_double(&bp,SECOND_LO_FREQUENCY,get_second_LO(demod));
  encode_double(&bp,SHIFT_FREQUENCY,demod->shift.freq);
  
  // Front end
  encode_double(&bp,FIRST_LO_FREQUENCY,demod->sdr.status.frequency);
  encode_byte(&bp,LNA_GAIN,demod->sdr.status.lna_gain);
  encode_byte(&bp,MIXER_GAIN,demod->sdr.status.mixer_gain);
  encode_byte(&bp,IF_GAIN,demod->sdr.status.if_gain);
  encode_float(&bp,DC_I_OFFSET,demod->sdr.DC_i);
  encode_float(&bp,DC_Q_OFFSET,demod->sdr.DC_q);
  encode_float(&bp,IQ_IMBALANCE,demod->sdr.imbalance);
  encode_float(&bp,IQ_PHASE,demod->sdr.sinphi);
  encode_double(&bp,CALIBRATE,demod->sdr.calibration);
  
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
    encode_float(&bp,NOISE_BANDWIDTH,demod->input.samprate * demod->filter.out->noise_gain);
  
  // Signals - these ALWAYS change
  encode_float(&bp,IF_POWER,demod->sig.if_power);
  encode_float(&bp,BASEBAND_POWER,demod->sig.bb_power);
  encode_float(&bp,NOISE_DENSITY,demod->sig.n0);
  
  // Demodulation mode
  enum demod_type demod_type = Demodtab[demod->demod_type].demod_type;
  encode_byte(&bp,DEMOD_TYPE,demod_type);
  switch(demod_type){
  case AM_DEMOD:
    encode_float(&bp,DEMOD_GAIN,demod->agc.gain);
    break;
  case FM_DEMOD:
    encode_float(&bp,PEAK_DEVIATION,demod->sig.pdeviation);
    encode_float(&bp,PL_TONE,demod->sig.plfreq);
    encode_float(&bp,FREQ_OFFSET,demod->sig.foffset);
    encode_float(&bp,DEMOD_SNR,demod->sig.snr);
    encode_byte(&bp,FM_FLAT,demod->opt.flat);
    break;
  case LINEAR_DEMOD:
    encode_float(&bp,DEMOD_GAIN,demod->agc.gain);
    encode_int32(&bp,INDEPENDENT_SIDEBAND,demod->filter.isb);
    encode_byte(&bp,PLL_ENABLE,demod->opt.pll);
    if(demod->opt.pll){
      encode_float(&bp,FREQ_OFFSET,demod->sig.foffset);
      encode_float(&bp,PLL_PHASE,demod->sig.cphase);
      encode_float(&bp,DEMOD_SNR,demod->sig.snr);
      encode_byte(&bp,PLL_LOCK,demod->sig.pll_lock);
      encode_byte(&bp,PLL_SQUARE,demod->opt.square);
      }
    break;
  }
  encode_int32(&bp,OUTPUT_CHANNELS,demod->output.channels);
  encode_eol(&bp);
  
  int len = compact_packet(&State[0],packet,full);
  send(demod->output.status_fd,packet,len,0);
  demod->output.metadata_packets++;
}

void decode_radio_commands(struct demod *demod,unsigned char *buffer,int length){
  unsigned char *cp = buffer;

  while(cp - buffer < length){
    enum status_type type = *cp++; // increment cp to length field
    
    if(type == EOL)
      break; // End of list
    
    unsigned int optlen = *cp++;
    if(cp - buffer + optlen >= length)
      break; // Invalid length
    
    double samptime = 1./demod->output.samprate;
    double nval;
    int i;

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
      demod->sdr.calibration = decode_double(cp,optlen);
      break;
    case RADIO_FREQUENCY:
      demod->tune.freq = decode_double(cp,optlen);
      set_freq(demod,demod->tune.freq,NAN);
      break;
    case FIRST_LO_FREQUENCY:
      nval = decode_double(cp,optlen);
      double new_lo2 = nval - get_freq(demod);
      if(LO2_in_range(demod,new_lo2,0))
	set_freq(demod,get_freq(demod),new_lo2);

      break;
    case SECOND_LO_FREQUENCY:
      nval = decode_double(cp,optlen);
      if(LO2_in_range(demod,nval,0)){
	// Only if valid
	demod->tune.freq += nval - get_second_LO(demod);
	set_freq(demod,demod->tune.freq,nval);
      }
      break;
    case LOW_EDGE:
      demod->filter.low = decode_float(cp,optlen);
      set_filter(demod->filter.out,samptime*demod->filter.low,samptime*demod->filter.high,demod->filter.kaiser_beta);
      break;
    case HIGH_EDGE:
      demod->filter.high = decode_float(cp,optlen);
      set_filter(demod->filter.out,samptime*demod->filter.low,samptime*demod->filter.high,demod->filter.kaiser_beta);      
      break;
    case SHIFT_FREQUENCY:
      demod->tune.shift = decode_double(cp,optlen);
      set_shift(demod,demod->tune.shift);
      break;
    case KAISER_BETA:
      demod->filter.kaiser_beta = decode_float(cp,optlen);
      if(demod->filter.kaiser_beta < 0)
	demod->filter.kaiser_beta = 0;
      set_filter(demod->filter.out,samptime*demod->filter.low,samptime*demod->filter.high,demod->filter.kaiser_beta);            
      break;
    case INDEPENDENT_SIDEBAND:
      demod->filter.isb = decode_int(cp,optlen);
      break;
    case PLL_ENABLE:
      demod->opt.pll = decode_int(cp,optlen);
      break;
    case PLL_SQUARE:
      demod->opt.square = decode_int(cp,optlen);
      break;
    case FM_FLAT:
      demod->opt.flat = decode_int(cp,optlen);
      break;
    case OUTPUT_CHANNELS:
      demod->output.channels = decode_int(cp,optlen);
      break;
    case COMMAND_TAG:
      demod->output.command_tag = decode_int(cp,optlen);
      break;
    default:
      break;
    }
    cp += optlen;
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
  int gainchange = 0;
  int nsamprate = 0;

  while(cp - buffer < length){
    enum status_type type = *cp++; // increment cp to length field

    if(type == EOL)
      break; // End of list

    unsigned int optlen = *cp++;
    if(cp - buffer + optlen >= length)
      break; // Invalid length
    switch(type){
    case EOL: // Shouldn't get here since it's checked above
      goto done;
    case OUTPUT_DATA_DEST_SOCKET:
      // SDR data destination address (usually multicast)
      // Becomes our data input socket
      if(optlen == 6){
	struct sockaddr_in *sin;
	sin = (struct sockaddr_in *)&demod->input.data_dest_address;
	sin->sin_family = AF_INET;
	memcpy(&sin->sin_addr.s_addr,cp,4);
	memcpy(&sin->sin_port,cp+4,2);
      } else if(optlen == 10){
	struct sockaddr_in6 *sin6;
	sin6 = (struct sockaddr_in6 *)&demod->input.data_dest_address;
	sin6->sin6_family = AF_INET6;
	memcpy(&sin6->sin6_addr,cp,8);
	memcpy(&sin6->sin6_port,cp+8,2);
      }
      break;
    case RADIO_FREQUENCY:
      nfreq = decode_double(cp,optlen);
      break;
    case OUTPUT_SAMPRATE:
      nsamprate = decode_int(cp,optlen);
      if(nsamprate != demod->sdr.status.samprate){
	demod->input.samprate = demod->sdr.status.samprate = nsamprate;
	demod->filter.decimate = demod->sdr.status.samprate / demod->output.samprate;
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
    case LOW_EDGE:
      demod->sdr.min_IF = decode_float(cp,optlen);
      break;
    case HIGH_EDGE:
      demod->sdr.max_IF = decode_float(cp,optlen);
      break;
    case LNA_GAIN:
      demod->sdr.status.lna_gain = decode_int(cp,optlen);
      gainchange++;
      break;
    case MIXER_GAIN:
      demod->sdr.status.mixer_gain = decode_int(cp,optlen);
      gainchange++;
      break;
    case IF_GAIN:
      demod->sdr.status.if_gain = decode_int(cp,optlen);
      gainchange++;
      break;
    case DC_I_OFFSET:
      demod->sdr.DC_i = decode_float(cp,optlen);
      break;
    case DC_Q_OFFSET:
      demod->sdr.DC_q = decode_float(cp,optlen);
      break;
    case IQ_IMBALANCE:
      demod->sdr.imbalance = decode_float(cp,optlen);
      break;
    case IQ_PHASE:
      demod->sdr.sinphi = decode_float(cp,optlen);
      break;
    case CALIBRATE:
      demod->sdr.calibration = decode_double(cp,optlen);
      break;
    default:
      break;
    }
    cp += optlen;
  }
  if(gainchange)
    demod->sdr.gain_factor = powf(10.,-0.05*(demod->sdr.status.lna_gain + demod->sdr.status.if_gain + demod->sdr.status.mixer_gain));
  if(!isnan(nfreq) && demod->sdr.status.frequency != nfreq && demod->sdr.status.samprate != 0){
    // Recalculate LO2
    demod->sdr.status.frequency = nfreq;
    double new_LO2 = -(demod->tune.freq - get_first_LO(demod));
    set_second_LO(demod,new_LO2);
  }
  done:;
}



  
