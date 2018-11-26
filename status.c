// $Id: status.c,v 1.4 2018/11/25 03:01:34 karn Exp karn $
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

int encode_object(unsigned char **buf,enum status_type type,void *data,int len){
  assert(buf != NULL);
  
  unsigned char *bp = *buf;
  assert(bp != NULL);

  *bp++ = type;
  unsigned char *cp = data;

  // Remove leading zero bytes
  while(*cp == 0 && len != 0){
    len--;
    cp++;
  }

  *bp++ = len;
  memcpy(bp,cp,len);
  *buf = bp + len;
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
  return encode_object(buf,type,&x,sizeof(x));
}

int encode_int16(unsigned char **buf,enum status_type type,uint16_t x){
  unsigned char data[2];

  // Byteswap
  data[0] = x >> 8;
  data[1] = x;
  return encode_object(buf,type,data,sizeof(data));
}

int encode_int32(unsigned char **buf,enum status_type type,uint32_t x){
  unsigned char data[4];

  // Byteswap
  data[0] = x >> 24;
  data[1] = x >> 16;
  data[2] = x >> 8;
  data[3] = x;
  return encode_object(buf,type,data,sizeof(data));
}

int encode_float(unsigned char **buf,enum status_type type,float x){
  uint32_t data;

  memcpy(&data,&x,sizeof(data));
  return encode_int32(buf,type,data);
}

int encode_int64(unsigned char **buf,enum status_type type,uint64_t x){
  unsigned char data[8];
  // Byteswap
  for(int i=0;i<8;i++)
    data[i] = x >> (56-8*i);

  return encode_object(buf,type,data,sizeof(data));
}
int encode_double(unsigned char **buf,enum status_type type,double x){
  uint64_t data;
  memcpy(&data,&x,sizeof(data));
  return encode_int64(buf,type,data);
}

// Decode byte string without byte swapping
unsigned char *decode_string(unsigned char **bp,unsigned char *buf,int buflen){
  unsigned char *cp = *bp;
  int len = *cp++;
  memcpy(buf,cp,min(len,buflen));
  *bp = cp + len;
  return buf;
}


// Decode encoded variable-length integers
// Works for byte, short, long, long long
uint64_t decode_int(unsigned char **bp){
  unsigned char *cp = *bp;
  int len = *cp++;
  uint64_t result = 0;
  // cp now points to beginning of abbreviated int
  // Byte swap as we accumulate
  while(len-- > 0)
    result = (result << 8) | *cp++;

  *bp = cp;
  return result;
}

float decode_float(unsigned char **bp){
  uint32_t result = decode_int(bp);
  return *(float *)&result;
}

double decode_double(unsigned char **bp){
  uint64_t result = decode_int(bp);
  return *(double *)&result;
}

