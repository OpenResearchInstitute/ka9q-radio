#ifndef _RTP_H
#define RTP_H 1
struct rtp_header {
  unsigned char vpxcc;
  unsigned char mpt;
  unsigned short seq;
  unsigned long timestamp;
  unsigned long ssrc;
};
#define RTP_VERS 2

#endif
