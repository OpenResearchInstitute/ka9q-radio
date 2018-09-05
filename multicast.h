// $Id: multicast.h,v 1.18 2018/08/04 21:06:16 karn Exp karn $
// Multicast and RTP functions, constants and structures
// Not every RTP module uses these yet, they need to be revised
// Copyright 2018, Phil Karn, KA9Q

#ifndef _MULTICAST_H
#define _MULTICAST_H 1
#include <stdint.h>

int setup_mcast(char const *target,int output);
extern char Default_mcast_port[];
extern int Mcast_ttl;

// Internal representation of RTP header -- NOT what's on wire!
struct rtp_header {
  int version;
  uint8_t type;
  uint16_t seq;
  uint32_t timestamp;
  uint32_t ssrc;
  int marker:1;
  int pad:1;
  int extension:1;
  int cc;
  uint32_t csrc[15];
};
#define RTP_MIN_SIZE 12  // min size of RTP header
#define RTP_VERS 2
#define RTP_MARKER 0x80  // Marker flag in mpt field

#define IQ_PT (97)    // NON-standard payload type for my raw I/Q stream - 16 bit version
#define IQ_PT8 (98)   // NON-standard payload type for my raw I/Q stream - 8 bit version
#define AX25_PT (96)  // NON-standard paylaod type for my raw AX.25 frames
#define PCM_MONO_PT (11)
#define PCM_STEREO_PT (10)

#define OPUS_PT (111) // Hard-coded NON-standard payload type for OPUS (should be dynamic with sdp)

// Convert between internal and wire representations of RTP header
unsigned char *ntoh_rtp(struct rtp_header *,unsigned char *);
unsigned char *hton_rtp(unsigned char *, struct rtp_header *);

// Internal state of common RTP receiver module
struct rtp_state {
  uint32_t ssrc;
  int init;
  uint16_t expected_seq;
  uint32_t expected_timestamp;
  long long packets;
  long long drops;
  long long dupes;
};

// Function to process incoming RTP packet headers
// Returns number of samples dropped or skipped by silence suppression, if any
int rtp_process(struct rtp_state *state,struct rtp_header *rtp,int samples);

// Utility routines for reading from, and writing integers to, network format in char buffers
static inline unsigned short get8(unsigned char const *dp){
  return *dp;
}

static inline unsigned short get16(unsigned char const *dp){
  return dp[0] << 8 | dp[1];
}

static inline unsigned long get24(unsigned char const *dp){
  return dp[0] << 16 | dp[1] << 8 | dp[2];
}

static inline unsigned long get32(unsigned char const *dp){
  return dp[0] << 24 | dp[1] << 16 | dp[2] << 8 | dp[3];
}

static inline unsigned char *put8(unsigned char *dp,uint8_t x){
  *dp++ = x;
  return dp;
}

static inline unsigned char *put16(unsigned char *dp,uint16_t x){
  *dp++ = x >> 8;
  *dp++ = x;
  return dp;
}

static inline unsigned char *put24(unsigned char *dp,uint32_t x){
  *dp++ = x >> 16;
  *dp++ = x >> 8;
  *dp++ = x;
  return dp;
}

static inline unsigned char *put32(unsigned char *dp,uint32_t x){
  *dp++ = x >> 24;
  *dp++ = x >> 16;
  *dp++ = x >> 8;
  *dp++ = x;
  return dp;
}


#endif
