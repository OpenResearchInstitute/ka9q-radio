#ifndef _RTP_H
#define RTP_H 1
#include <stdint.h>
struct rtp_header {
  uint8_t vpxcc;
  uint8_t mpt;
  uint16_t seq;
  uint32_t timestamp;
  uint32_t ssrc;
};
#define RTP_VERS 2



#endif
