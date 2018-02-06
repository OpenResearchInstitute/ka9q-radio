#ifndef _MULTICAST_H
#define _MULTICAST_H
int setup_mcast(char const *target,int output);
extern char Default_mcast_port[];
extern int Mcast_ttl;

struct rtp_header {
  uint8_t vpxcc;
  uint8_t mpt;
  uint16_t seq;
  uint32_t timestamp;
  uint32_t ssrc;
};
#define RTP_VERS 2


#endif
