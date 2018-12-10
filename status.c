// $Id: status.c,v 1.9 2018/12/04 04:33:18 karn Exp karn $
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
#include "status.h"

// Encode 64-bit integer, byte swapped, leading zeroes suppressed
int encode_int64(unsigned char **buf,enum status_type type,uint64_t x){
  unsigned char *cp = *buf;

  *cp++ = type;

  int len = sizeof(x);
  while(len > 0 && (x & 0xff00000000000000LL) == 0){
    x <<= 8;
    len--;
  }
  *cp++ = len;

  for(int i=0; i<len; i++){
    *cp++ = x >> 56;
    x <<= 8;
  }

  *buf = cp;
  return 2+len;
}


// Single null type byte means end of list
int encode_eol(unsigned char **buf){
  unsigned char *bp = *buf;

  *bp++ = EOL;
  *buf = bp;
  return 1;
}

int encode_byte(unsigned char **buf,enum status_type type,unsigned char x){
  unsigned char *cp = *buf;
  *cp++ = type;
  *cp++ = sizeof(x);
  *cp++ = x;
  *buf = cp;
  return 2+sizeof(x);
}

int encode_int16(unsigned char **buf,enum status_type type,uint16_t x){
  return encode_int64(buf,type,(uint64_t)x);
}

int encode_int32(unsigned char **buf,enum status_type type,uint32_t x){
  return encode_int64(buf,type,(uint64_t)x);
}

int encode_int(unsigned char **buf,enum status_type type,int x){
  return encode_int64(buf,type,(int)x);
}


int encode_float(unsigned char **buf,enum status_type type,float x){
  uint32_t data;

  memcpy(&data,&x,sizeof(data));
  return encode_int32(buf,type,(uint64_t)data);
}

int encode_double(unsigned char **buf,enum status_type type,double x){
  uint64_t data;
  memcpy(&data,&x,sizeof(data));
  return encode_int64(buf,type,data);
}

// Encode byte string without byte swapping
int encode_string(unsigned char **bp,enum status_type type,void *buf,int buflen){
  unsigned char *cp = *bp;
  *cp++ = type;
  if(buflen > 255)
    buflen = 255;
  *cp++ = buflen;
  memcpy(cp,buf,buflen);
  *bp = cp + buflen;
  return 2+buflen;
}


// Decode byte string without byte swapping
void *decode_string(unsigned char **bp,void *buf,int buflen){
  unsigned char *cp = *bp;
  int len = *cp++;
  memcpy(buf,cp,min(len,buflen));
  *bp = cp + len;
  return buf;
}


// Decode encoded variable-length UNSIGNED integers
// At entry, *bp -> length field (not type!)
// Works for byte, short, long, long long
uint64_t decode_int(unsigned char *cp,int len){
  uint64_t result = 0;
  // cp now points to beginning of abbreviated int
  // Byte swap as we accumulate
  while(len-- > 0)
    result = (result << 8) | *cp++;

  return result;
}

float decode_float(unsigned char *cp,int len){
  if(len == 8)
    return (float)decode_double(cp,len);

  uint32_t result = decode_int(cp,len);
  return *(float *)&result;
}

double decode_double(unsigned char *cp,int len){
  if(len == 4)
    return (double)decode_float(cp,len);

  uint64_t result = decode_int(cp,len);
  return *(double *)&result;
}

int compact_packet(struct state *s,unsigned char *pkt,int force){
  unsigned char *input = pkt;
  unsigned char *output = pkt;

  *output++ = *input++; // command/response byte (don't really have to copy)

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
void decode_socket(char *host,char *port,unsigned char *val,int optlen){
  struct sockaddr_storage sock;
  struct sockaddr_in *sin = (struct sockaddr_in *)&sock;
  struct sockaddr_in6 *sin6 = (struct sockaddr_in6 *)&sock;

  if(optlen == 6){
    sin->sin_family = AF_INET;
    memcpy(&sin->sin_addr.s_addr,val,4);
    memcpy(&sin->sin_port,val+4,2);
  } else if(optlen == 10){
    sin6->sin6_family = AF_INET6;
    memcpy(&sin6->sin6_addr,val,8);
    memcpy(&sin6->sin6_port,val+8,2);
  } 
  getnameinfo((struct sockaddr *)&sock,sizeof(sock),host,NI_MAXHOST,port,NI_MAXSERV,
	      NI_NOFQDN|NI_NUMERICHOST|NI_NUMERICSERV);

}


void dump_radio_status(unsigned char *buffer,int length){
  unsigned char *cp = buffer;
  char host[NI_MAXHOST];
  char port[NI_MAXSERV];
  int i;

  while(cp - buffer < length){
    enum status_type type = *cp++; // increment cp to length field
    
    if(type == EOL)
      break; // End of list

    unsigned int optlen = *cp++;
    if(cp - buffer + optlen >= length)
      break; // Invalid length
    switch(type){
    case EOL: // Shouldn't get here
      goto done;
    case COMMAND_TAG:
      printf(" cmd tag %lx;",decode_int(cp,optlen));
      break;
    case COMMANDS:
      printf(" commands %'lu;",decode_int(cp,optlen));
      break;
    case GPS_TIME:
      printf(" time %s;",lltime(decode_int(cp,optlen)));
      break;
    case INPUT_DATA_SOURCE_SOCKET:
      decode_socket(host,port,cp,optlen);
      printf(" in data src %s:%s;",host,port);
      break;
    case INPUT_METADATA_SOURCE_SOCKET:
      decode_socket(host,port,cp,optlen);
      printf(" in metadata src %s:%s;",host,port);
      break;
    case INPUT_DATA_DEST_SOCKET:
      decode_socket(host,port,cp,optlen);      
      printf(" in data dst %s:%s;",host,port);
      break;
    case INPUT_METADATA_DEST_SOCKET:      
      decode_socket(host,port,cp,optlen);      
      printf(" in metadata dst %s:%s;",host,port);
      break;
    case INPUT_SSRC:
      printf(" in SSRC %lx;",decode_int(cp,optlen));
      break;
    case INPUT_SAMPRATE:
      printf(" in samprate %'lu Hz;",decode_int(cp,optlen));
      break;
    case INPUT_DATA_PACKETS:
      printf(" in data packets %'lu",decode_int(cp,optlen));
      break;
    case INPUT_METADATA_PACKETS:
      printf(" in metadata packets %'lu;",decode_int(cp,optlen));
      break;
    case INPUT_SAMPLES:
      printf(" in samples %'lu;",decode_int(cp,optlen));
      break;
    case INPUT_DROPS:
      printf(" in drops %'lu;",decode_int(cp,optlen));
      break;
    case INPUT_DUPES:
      printf(" in dupes %'lu;",decode_int(cp,optlen));
      break;
    case OUTPUT_DATA_SOURCE_SOCKET:
      decode_socket(host,port,cp,optlen);      
      printf(" out data src %s:%s;",host,port);
      break;
    case OUTPUT_DATA_DEST_SOCKET:
      decode_socket(host,port,cp,optlen);      
      printf(" out data dst %s:%s;",host,port);
      break;
    case OUTPUT_SSRC:
      printf(" out SSRC %lx;",decode_int(cp,optlen));
      break;
    case OUTPUT_TTL:
      printf(" out TTL %'lu;",decode_int(cp,optlen));
      break;
    case OUTPUT_SAMPRATE:
      printf(" out samprate %'lu Hz;",decode_int(cp,optlen));
      break;
    case OUTPUT_DATA_PACKETS:
      printf(" out data pkts %'lu;",decode_int(cp,optlen));
      break;
    case OUTPUT_METADATA_PACKETS:
      printf(" out metadata pkts %'lu;",decode_int(cp,optlen));
      break;
    case RADIO_FREQUENCY:
      printf(" radio %lg Hz;",decode_double(cp,optlen));
      break;
    case FIRST_LO_FREQUENCY:
      printf(" first LO %lg Hz;",decode_double(cp,optlen));
      break;
    case SECOND_LO_FREQUENCY:
      printf(" second LO %lg Hz;",decode_double(cp,optlen));
      break;
    case SHIFT_FREQUENCY:
      printf(" shift %lg Hz;",decode_double(cp,optlen));
      break;
    case DOPPLER_FREQUENCY:
      printf(" doppler %lg Hz;",decode_double(cp,optlen));
      break;
    case DOPPLER_FREQUENCY_RATE:
      printf(" doppler rate %lg Hz/s;",decode_double(cp,optlen));
      break;
    case LNA_GAIN:
      printf(" lna gain %'lu dB;",decode_int(cp,optlen));
      break;
    case MIXER_GAIN:
      printf(" mixer gain %'lu dB;",decode_int(cp,optlen));
      break;
    case IF_GAIN:
      printf(" if gain %'lu dB;",decode_int(cp,optlen));
      break;
    case DC_I_OFFSET:
      printf(" DC I offset %g;",decode_float(cp,optlen));
      break;
    case DC_Q_OFFSET:
      printf(" DC Q offset %g;",decode_float(cp,optlen));
      break;
    case IQ_IMBALANCE:
      printf(" Gain imbal %.1f dB;",10*log10f(decode_float(cp,optlen)));
      break;
    case IQ_PHASE:
      printf(" phase imbal %.1f deg;",(180./M_PI)*asinf(decode_float(cp,optlen)));
      break;
    case LOW_EDGE:
      printf(" filt low %g Hz;",decode_float(cp,optlen));
      break;
    case HIGH_EDGE:
      printf(" filt high %g Hz;",decode_float(cp,optlen));
      break;
    case KAISER_BETA:
      printf(" filter kaiser %g;",decode_float(cp,optlen));      
      break;
    case FILTER_BLOCKSIZE:
      printf(" filter L %'lu;",decode_int(cp,optlen));
      break;
    case FILTER_FIR_LENGTH:
      printf(" filter M %'lu;",decode_int(cp,optlen));
      break;
    case NOISE_BANDWIDTH:
      printf(" noise BW %g Hz;",decode_float(cp,optlen));
      break;
    case IF_POWER:
      printf(" IF pwr %.1f dBFS;",10*log10f(decode_float(cp,optlen)));
      break;
    case BASEBAND_POWER:
      printf(" BB pwr %.1f dBFS;",10*log10f(decode_float(cp,optlen)));
      break;
    case NOISE_DENSITY:
      printf(" N0 %.1f dB/Hz;",10*log10f(decode_float(cp,optlen)));
      break;
    case DEMOD_TYPE:
      i = decode_int(cp,optlen); // ????
      printf(" demod %d;",i);
      break;
    case INDEPENDENT_SIDEBAND:
      printf(" ISB %'lu;",decode_int(cp,optlen));
      break;
    case DEMOD_SNR:
      printf(" Demod SNR %.1f dB;",10*log10f(decode_float(cp,optlen)));
      break;
    case DEMOD_GAIN:
      printf(" AGC gain %.1f dB;",10*log10(decode_float(cp,optlen)));
      break;
    case FREQ_OFFSET:
      printf(" freq offset %g Hz;",decode_float(cp,optlen));
      break;
    case PEAK_DEVIATION:
      printf(" peak FM dev %g Hz;",decode_float(cp,optlen));
      break;
    case PL_TONE:
      printf(" PL tone %g Hz;",decode_float(cp,optlen));
      break;
    case PLL_LOCK:
      printf(" PLL lock %'lu;",decode_int(cp,optlen));
      break;
    case PLL_ENABLE:
      printf(" PLL enable %'lu;",decode_int(cp,optlen));
      break;
    case PLL_SQUARE:
      printf(" PLL square %'lu;",decode_int(cp,optlen));
      break;
    case PLL_PHASE:
      printf(" PLL phase %g deg;",(180/M_PI)*decode_float(cp,optlen));
      break;
    case OUTPUT_CHANNELS:
      printf(" out channels %'lu;",decode_int(cp,optlen));
      break;
    case CALIBRATE:
      printf(" calibration %lg;",decode_double(cp,optlen));
      break;
    default:
      break;
    }
    cp += optlen;
  }
 done:;
  printf("\n");
}
