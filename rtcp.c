// $Id$
// Real Time Control Protocol (RTCP)
// Sep 2018 Phil Karn, KA9Q
#include <sys/types.h>
#include <string.h>
#include "multicast.h"

#define UNIX_EPOCH 2208988800UL // Seconds between Jan 1 1900 and Jan 1 1970

// Internal format of sender report segment
struct rtcp_sr {
  unsigned int ssrc;
  long long ntp_timestamp;
  unsigned int rtp_timestamp;
  unsigned int packet_count;
  unsigned int byte_count;
};

// Internal format of receiver report segment

struct rtcp_rr {
  unsigned int ssrc;
  int lost_fract;
  int lost_packets;
  int highest_seq;
  int jitter;
  int lsr; // Last SR
  int dlsr; // Delay since last SR
};

// Build a RTCP sender report in network order
// Return pointer to byte after end of written packet
unsigned char *gen_sr(unsigned char *output,int bufsize,struct rtcp_sr const *sr,
       struct rtcp_rr const *rr,int rc){

  int words = 4 + 6 + 6*rc;

  if(4*words > bufsize)
    return NULL; // Not enough room in buffer

  // SR packet header
  *output++ = (2 << 6) | rc;
  *output++ = 200;
  output = put16(output,words-1);

  // Sender info
  output = put32(output,sr->ssrc);
  output = put32(output,sr->ntp_timestamp >> 32);
  output = put32(output,sr->ntp_timestamp);
  output = put32(output,sr->rtp_timestamp);
  output = put32(output,sr->packet_count);
  output = put32(output,sr->byte_count);

  // Receiver info (if any)
  for(int i=0; i < rc; i++){
    output = put32(output,rr->ssrc);
    *output++ = rr->lost_fract;
    output = put24(output,rr->lost_packets);
    output = put32(output,rr->highest_seq);
    output = put32(output,rr->jitter);
    output = put32(output,rr->lsr);
    output = put32(output,rr->dlsr);
    rr++;
  }
  return output;
}
// Build a RTCP receiver report in network order
// Return pointer to byte after end of written packet
unsigned char *gen_rr(unsigned char *output,int bufsize,struct rtcp_rr const *rr,int rc){

  int words = 4 + 6*rc;

  if(4*words > bufsize)
    return NULL; // Not enough room in buffer

  // RR packet header
  *output++ = (2 << 6) | rc;
  *output++ = 201; // Receiver report
  output = put16(output,words-1);

  // Receiver info (if any)
  for(int i=0; i < rc; i++){
    output = put32(output,rr->ssrc);
    *output++ = rr->lost_fract;
    output = put24(output,rr->lost_packets);
    output = put32(output,rr->highest_seq);
    output = put32(output,rr->jitter);
    output = put32(output,rr->lsr);
    output = put32(output,rr->dlsr);
    rr++;
  }
  return output;
}

enum sdes_type {
  CNAME=1,
  NAME=2,
  EMAIL=3,
  PHONE=4,
  LOC=5,
  TOOL=6,
  NOTE=7,
  PRIV=8,
};

struct rtcp_sdes {
  enum sdes_type type;
  uint32_t ssrc;
  int mlen;
  char message[256];
};


// Build a RTCP source description packet in network order
// Return pointer to byte after end of written packet
unsigned char *gen_sdes(unsigned char *output,int bufsize,struct rtcp_sdes *sdes,int sc){
  
  if(sc < 0 || sc > 31) // Range check on source count
    return NULL;

  // Calculate size
  int bytes = 4 + 1; // header plus terminating null
  for(int i=0; i < sc; i++){
    if(sdes[i].mlen < 0 || sdes[i].mlen > 255)
      return NULL;
    bytes = 4 + sdes[i].mlen; // ssrc + items
  }
  // Round up to 4 byte boundary
  int words = (bytes + 3)/4;

  if(4*words > bufsize)
    return NULL;

  memset(output,0,bufsize); // easist way to guarantee nulls at end

  *output++ = (2 << 6) | sc;
  *output++ = 202; // SDES
  output = put16(output,words-1);

  // Put each item
  for(int i=0; i<sc; i++){
    output = put32(output,sdes[i].ssrc);
    *output++ = sdes[i].type;
    *output++ = sdes[i].mlen;
    memcpy(output,sdes[i].message,sdes[i].mlen); // Buffer overrun avoided by size calc?
    output += sdes[i].mlen;
  }
  return output;
}

unsigned char *gen_bye(unsigned char *output,int bufsize,uint32_t *ssrcs,int sc){
  if(sc < 0 || sc > 31) // Range check on source count
    return NULL;

  int words = 1 + sc;
  if(4*words > bufsize)
    return NULL;

  *output++ = (2 << 6) | sc;
  *output++ = 203; // BYE
  output = put16(output,words-1);

  for(int i=0; i<sc; i++)
    output = put32(output,ssrcs[i]);

  return output;
}
