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
#include "status.h"

// Previous transmitted state, used to detect changes
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


    encode_int16(&bp,TYPE,0); // Status response

    // Source information
    // Who's sending us information
    {
      struct sockaddr_in *sin;
      struct sockaddr_in6 *sin6;
      switch(demod->input_source_address.ss_family){
      case AF_INET:
	sin = (struct sockaddr_in *)&demod->input_source_address;
	encode_object(&bp,INPUT_SOURCE_ADDRESS,&sin->sin_addr.s_addr,4); // Already in network order
	encode_int16(&bp,INPUT_SOURCE_PORT,sin->sin_port);
	break;
      case AF_INET6:
	sin6 = (struct sockaddr_in6 *)&demod->input_source_address;
	encode_object(&bp,INPUT_SOURCE_ADDRESS,&sin6->sin6_addr,8);
	encode_int16(&bp,INPUT_SOURCE_PORT,sin6->sin6_port);
	break;
      default:
	break;
      }
    }
    // Destination address (usually multicast) and port on which we're getting input data
    {
      struct sockaddr_in *sin;
      struct sockaddr_in6 *sin6;
      struct sockaddr_storage s;
      socklen_t slen = sizeof(s);
      getsockname(demod->input_fd,(struct sockaddr *)&s,&slen);
      switch(s.ss_family){
      case AF_INET:
	sin = (struct sockaddr_in *)&s;
	encode_object(&bp,INPUT_DEST_ADDRESS,&sin->sin_addr.s_addr,4); // Already in network order
	encode_int16(&bp,INPUT_DEST_PORT,sin->sin_port);
	break;
      case AF_INET6:
	sin6 = (struct sockaddr_in6 *)&s;
	encode_object(&bp,INPUT_DEST_ADDRESS,(char *)&sin6->sin6_addr,8);
	encode_int16(&bp,INPUT_DEST_PORT,sin6->sin6_port);
	break;
      default:
	break;
      }
    }
    encode_int32(&bp,INPUT_SSRC,demod->rtp_state.ssrc);
    encode_int32(&bp,INPUT_SAMPRATE,demod->status.samprate);
    // Where we're sending output
    {
      struct sockaddr_in *sin;
      struct sockaddr_in6 *sin6;
      struct sockaddr_storage s;
      socklen_t slen = sizeof(s);
      getpeername(audio->audio_mcast_fd,(struct sockaddr *)&s,&slen);
      switch(s.ss_family){
      case AF_INET:
	sin = (struct sockaddr_in *)&s;
	encode_object(&bp,OUTPUT_DEST_ADDRESS,&sin->sin_addr.s_addr,4); // Already in network order
	encode_int16(&bp,OUTPUT_DEST_PORT,sin->sin_port);
	break;
      case AF_INET6:
	sin6 = (struct sockaddr_in6 *)&s;
	encode_object(&bp,OUTPUT_DEST_ADDRESS,&sin6->sin6_addr,8);
	encode_int16(&bp,OUTPUT_DEST_PORT,sin6->sin6_port);
	break;
      default:
	break;
      }
    }
    encode_int32(&bp,OUTPUT_SSRC,Audio.rtp.ssrc);
    encode_byte(&bp,OUTPUT_TTL,Mcast_ttl);
    encode_int32(&bp,OUTPUT_SAMPRATE,audio->samprate);
    encode_int64(&bp,INPUT_PACKETS,demod->rtp_state.packets);
    encode_int64(&bp,INPUT_SAMPLES,demod->samples);
    encode_int64(&bp,INPUT_DROPS,demod->rtp_state.drops);
    encode_int64(&bp,INPUT_DUPES,demod->rtp_state.dupes);
    encode_int64(&bp,OUTPUT_PACKETS,audio->rtp.packets);

    // Tuning
    encode_double(&bp,CENTER_FREQUENCY,get_freq(demod));
    encode_double(&bp,INTERMEDIATE_FREQUENCY,-get_second_LO(demod));
    encode_double(&bp,SHIFT_FREQUENCY,demod->shift);

    // Doppler info
    encode_double(&bp,DOPPLER_FREQUENCY,get_doppler(demod));
    encode_double(&bp,DOPPLER_FREQUENCY_RATE,get_doppler_rate(demod));

    // Filtering
    encode_float(&bp,LOW_EDGE,demod->low);
    encode_float(&bp,HIGH_EDGE,demod->high);
    encode_float(&bp,KAISER_BETA,demod->kaiser_beta);
    encode_int32(&bp,FILTER_BLOCKSIZE,demod->L);
    encode_int32(&bp,FILTER_FIR_LENGTH,demod->M);

    // Signals - these ALWAYS change
    encode_float(&bp,IF_POWER,demod->if_power);
    encode_float(&bp,BASEBAND_POWER,demod->bb_power);
    encode_float(&bp,NOISE_DENSITY,demod->n0);

    // Demodulation mode
    encode_object(&bp,RADIO_MODE,demod->mode,strlen(demod->mode));
    enum demod_type demod_type = Demodtab[demod->demod_index].demod_type;
    encode_byte(&bp,DEMOD_MODE,demod_type);
    switch(demod_type){
    case AM_DEMOD:
      encode_float(&bp,DEMOD_GAIN,demod->gain);
      break;
    case FM_DEMOD:
      encode_float(&bp,PEAK_DEVIATION,demod->pdeviation);
      encode_float(&bp,PL_TONE,demod->plfreq);
      encode_float(&bp,FREQ_OFFSET,demod->foffset);
      encode_float(&bp,DEMOD_SNR,demod->snr);
      break;
    case LINEAR_DEMOD:
      encode_float(&bp,DEMOD_GAIN,demod->gain);
      encode_byte(&bp,INDEPENDENT_SIDEBAND,(demod->flags & ISB)? 1:0);
      if(demod->flags & PLL){
	encode_float(&bp,FREQ_OFFSET,demod->foffset);
	encode_float(&bp,PLL_PHASE,demod->cphase);
	encode_float(&bp,DEMOD_SNR,demod->snr);
	encode_byte(&bp,PLL_LOCK,(demod->spare == 48000)? 1:0);
	encode_byte(&bp,PLL_SQUARE,(demod->flags & SQUARE)? 1:0);
      }
      break;
    }
    encode_byte(&bp,OUTPUT_CHANNELS,(demod->flags & MONO)? 1:2);
    encode_eol(&bp);

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


  
