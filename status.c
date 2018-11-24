// $Id: status.c,v 1.1 2018/11/22 09:58:05 karn Exp karn $
// Thread to emit receiver status packets
// Copyright 2018 Phil Karn, KA9Q

#define _GNU_SOURCE 1
#include <assert.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
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


enum status_type {
  EOL = 0,	  
  TYPE,
  INPUT_SOURCE_ADDRESS,
  INPUT_SOURCE_PORT,
  INPUT_DEST_ADDRESS,
  INPUT_DEST_PORT,
  INPUT_SSRC,
  INPUT_SAMPRATE,
  INPUT_PACKETS,
  INPUT_SAMPLES,
  INPUT_DROPS,
  INPUT_DUPES,

  OUTPUT_DEST_ADDRESS, // 12
  OUTPUT_DEST_PORT,
  OUTPUT_SSRC,
  OUTPUT_TTL,
  OUTPUT_SAMPRATE,
  OUTPUT_PACKETS,

  // Tuning
  CENTER_FREQUENCY, // 18
  INTERMEDIATE_FREQUENCY,
  SHIFT_FREQUENCY,
  DOPPLER_FREQUENCY,
  DOPPLER_FREQUENCY_RATE,

  // Hardware gains
  LNA_GAIN, // 23
  MIX_GAIN,
  IF_GAIN,

  // Filtering
  LOW_EDGE, // 26
  HIGH_EDGE,
  KAISER_BETA,
  FILTER_BLOCKSIZE,
  FILTER_FIR_LENGTH,

  // Signals
  IF_POWER, // 31
  BASEBAND_POWER,
  NOISE_DENSITY,

  // Demodulation
  DEMOD_MODE, // 1 = AM envelope, 2 = FM, 3 = linear // 34
  INDEPENDENT_SIDEBAND,
  OUTPUT_CHANNELS,
  DEMOD_SNR,
  DEMOD_GAIN,
  FREQ_OFFSET,    // FM, PLL linear

  // FM only
  PEAK_DEVIATION, // 40
  PL_TONE,
  
  // PLL linear only
  PLL_LOCK, // 43
  PLL_SQUARE,
  PLL_PHASE,
};


unsigned char *encode_eol(unsigned char *buf){
  *buf++ = EOL;
  return buf;
}


// Big-end TLV encoding, suppressing leading zeroes
unsigned char *encode_byte(unsigned char *buf,enum status_type type,unsigned char x){
  *buf++ = type;
  if(x != 0){
    *buf++ = sizeof(x);
    *buf++ = x;
  } else {
    *buf++ = 0;
  }
  return buf;
}

unsigned char *encode_buf(unsigned char *buf,enum status_type type,void *data,int len){
  *buf++ = type;
  *buf++ = len;
  memcpy(buf,data,len);
  return buf + len;
}

unsigned char *encode_int16(unsigned char *buf,enum status_type type,uint16_t x){
  *buf++ = type;
  *buf++ = sizeof(x);
  *buf++ = x >> 8;
  *buf++ = x;
  return buf;
}

unsigned char *encode_int32(unsigned char *buf,enum status_type type,uint32_t x){
  int size = sizeof(x);
  while((x >> 24) == 0 && size > 0){
    size--;
    x <<= 8;
  }
  *buf++ = type;
  *buf++ = size;
  while(size--){
    *buf++ = x >> 24;
    x <<= 8;
  }
  return buf;
}


unsigned char *encode_float(unsigned char *buf,enum status_type type,float x){
  return encode_int32(buf,type,(uint32_t)x);
}

unsigned char *encode_int64(unsigned char *buf,enum status_type type,uint64_t x){
  int size = sizeof(x);
  while((x >> 56) == 0 && size > 0){
    size--;
    x <<= 8;
  }
  *buf++ = type;
  *buf++ = size;
  while(size--){
    *buf++ = x >> 56;
    x <<= 8;
  }
  return buf;
}
unsigned char *encode_double(unsigned char *buf,enum status_type type,double x){
  return encode_int64(buf,type,(uint64_t)x);
}

struct state {
  int length;
  unsigned char value[256];
};



struct state State[256];
int compact_packet(struct state *s,unsigned char *pkt,int force);

// Thread to periodically transmit receiver state
void *status(void *arg){
  pthread_setname("status");
  assert(arg != NULL);
  struct demod * const demod = arg;
  struct audio * const audio = &Audio; // Eventually make parameter

  memset(State,0,sizeof(State));
  
  for(int count=0;;count++){
    if(audio->status_mcast_fd <= 0){
      usleep(1);
      continue;
    }

    // emit status packets indefinitely
    unsigned char packet[2048],*bp;
    memset(packet,0,sizeof(packet));
    bp = packet;

    struct sockaddr_storage s;
    socklen_t slen;
    struct sockaddr_in *sin;
    struct sockaddr_in6 *sin6;

    bp = encode_byte(bp,TYPE,1); // Status response

    // Source information
    // Who's sending us information
    switch(demod->input_source_address.ss_family){
    case AF_INET:
      sin = (struct sockaddr_in *)&demod->input_source_address;
      bp = encode_buf(bp,INPUT_SOURCE_ADDRESS,&sin->sin_addr.s_addr,4); // Already in network order
      bp = encode_int16(bp,INPUT_SOURCE_PORT,sin->sin_port);
      break;
    case AF_INET6:
      sin6 = (struct sockaddr_in6 *)&demod->input_source_address;
      bp = encode_buf(bp,INPUT_SOURCE_ADDRESS,&sin6->sin6_addr,8);
      bp = encode_int16(bp,INPUT_SOURCE_PORT,sin6->sin6_port);
      break;
    default:
      break;
    }
    // Destination address (usually multicast) and port on which we're getting input data
    slen = sizeof(s);
    getsockname(demod->input_fd,(struct sockaddr *)&s,&slen);
    
    switch(s.ss_family){
    case AF_INET:
      sin = (struct sockaddr_in *)&s;
      bp = encode_buf(bp,INPUT_DEST_ADDRESS,&sin->sin_addr.s_addr,4); // Already in network order
      bp = encode_int16(bp,INPUT_DEST_PORT,sin->sin_port);
      break;
    case AF_INET6:
      sin6 = (struct sockaddr_in6 *)&s;
      bp = encode_buf(bp,INPUT_DEST_ADDRESS,(char *)&sin6->sin6_addr,8);
      bp = encode_int16(bp,INPUT_DEST_PORT,sin6->sin6_port);
      break;
    default:
      break;
    }
    bp = encode_int32(bp,INPUT_SSRC,demod->rtp_state.ssrc);
    bp = encode_float(bp,INPUT_SAMPRATE,(float)demod->status.samprate);
    // Where we're sending output
    slen = sizeof(s);
    getpeername(audio->audio_mcast_fd,(struct sockaddr *)&s,&slen);
    switch(s.ss_family){
    case AF_INET:
      sin = (struct sockaddr_in *)&s;
      bp = encode_buf(bp,OUTPUT_DEST_ADDRESS,&sin->sin_addr.s_addr,4); // Already in network order
      bp = encode_int16(bp,OUTPUT_DEST_PORT,sin->sin_port);
      break;
    case AF_INET6:
      sin6 = (struct sockaddr_in6 *)&s;
      bp = encode_buf(bp,OUTPUT_DEST_ADDRESS,&sin6->sin6_addr,8);
      bp = encode_int16(bp,OUTPUT_DEST_PORT,sin6->sin6_port);
      break;
    default:
      break;
    }
    bp = encode_int32(bp,OUTPUT_SSRC,Audio.rtp.ssrc);
    bp = encode_byte(bp,OUTPUT_TTL,Mcast_ttl);
    bp = encode_float(bp,OUTPUT_SAMPRATE,(float)audio->samprate);
    bp = encode_int64(bp,INPUT_PACKETS,demod->rtp_state.packets);
    bp = encode_int64(bp,INPUT_SAMPLES,demod->samples);
    bp = encode_int64(bp,INPUT_DROPS,demod->rtp_state.drops);
    bp = encode_int64(bp,INPUT_DUPES,demod->rtp_state.dupes);
    bp = encode_int64(bp,OUTPUT_PACKETS,audio->rtp.packets);

    // Tuning
    bp = encode_float(bp,CENTER_FREQUENCY,get_freq(demod));
    bp = encode_float(bp,INTERMEDIATE_FREQUENCY,-demod->second_LO);
    bp = encode_float(bp,SHIFT_FREQUENCY,demod->shift);

    // Doppler info
    bp = encode_float(bp,DOPPLER_FREQUENCY,get_doppler(demod));
    bp = encode_float(bp,DOPPLER_FREQUENCY_RATE,get_doppler_rate(demod));

    // Filtering
    bp = encode_float(bp,LOW_EDGE,demod->low);
    bp = encode_float(bp,HIGH_EDGE,demod->high);
    bp = encode_float(bp,KAISER_BETA,demod->kaiser_beta);
    bp = encode_float(bp,FILTER_BLOCKSIZE,demod->L);
    bp = encode_float(bp,FILTER_FIR_LENGTH,demod->M);

    // Signals - these ALWAYS change
    bp = encode_float(bp,IF_POWER,demod->if_power);
    bp = encode_float(bp,BASEBAND_POWER,demod->bb_power);
    bp = encode_float(bp,NOISE_DENSITY,demod->n0);

    // Demodulation
    // This needs to be MUCH cleaner
    int dm = 0;
    if(strcmp(demod->demod_name,"AM") == 0)
      dm = 1;
    else if(strcmp(demod->demod_name,"FM") == 0)
      dm = 2;
    else
      dm = 3;
    bp = encode_byte(bp,DEMOD_MODE,dm);
	
    if(demod->flags & ISB)
      bp = encode_byte(bp,INDEPENDENT_SIDEBAND,1);
    else
      bp = encode_byte(bp,INDEPENDENT_SIDEBAND,0);      

    if(demod->flags & MONO)
      bp = encode_byte(bp,OUTPUT_CHANNELS,1);
    else
      bp = encode_byte(bp,OUTPUT_CHANNELS,2);      

    bp = encode_float(bp,DEMOD_SNR,demod->snr);

    bp = encode_float(bp,DEMOD_GAIN,demod->gain);

    bp = encode_float(bp,FREQ_OFFSET,demod->foffset);

    bp = encode_float(bp,PEAK_DEVIATION,demod->pdeviation);

    bp = encode_float(bp,PL_TONE,demod->plfreq);
    // Linear
    if(demod->flags & PLL){
      if(demod->spare == 48000)
	bp = encode_byte(bp,PLL_LOCK,1);
      else
	bp = encode_byte(bp,PLL_LOCK,0);      
      if(demod->flags & SQUARE)
	bp = encode_byte(bp,PLL_SQUARE,1);
      if(!isnan(demod->cphase))
	bp = encode_float(bp,PLL_PHASE,demod->cphase*DEGPRA);
    }
    bp = encode_eol(bp);
    int len = compact_packet(&State[0],packet,(count % 10) == 0);

    send(audio->status_mcast_fd,packet,len,0);

    usleep(100000);
  }
}



int compact_packet(struct state *s,unsigned char *pkt,int force){
  
  unsigned char *input = pkt;
  unsigned char *output = pkt;

  // Read new packet into table, copying elements that have changed to output
  while(1){
    int type = *input++;
    if(type == EOL)
      break;
    int len = *input++;
    assert(type >= 0 && type < 256);
    assert(len >= 0 && len < 256);    
    if(force || s[type].length != len || memcmp(s[type].value,input,len) != 0){
      s[type].length = len;
      memcpy(s[type].value,input,len);
      *output++ = type;
      *output++ = len;
      assert(output <= input);
      memmove(output,input,len);
      output += len;
    }
    input += len;
  }
  *output++ = EOL;
  return output - pkt;
}


  
