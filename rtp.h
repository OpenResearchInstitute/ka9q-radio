#ifndef _RTP_H
#define RTP_H 1
#include <stdint.h>
// Real Time Protocol header

struct rtp_header {
  uint8_t vpxcc;
  uint8_t mpt;
  uint16_t seq;
  uint32_t timestamp;
  uint32_t ssrc;
};
#define RTP_VERS 2

#define UNIX_EPOCH 2208988800UL // 1970-1900 in seconds

// RTP Control Protocol (RTCP) header for sender reports
struct rtcp_sr_header {
  uint8_t vprc; // = RTP_VERS << 6
  uint8_t pt; // = 200 (SR)
  uint16_t length;
  uint32_t ssrc;
  uint32_t ntp_msw;
  uint32_t rtp_lsw;
  uint32_t timestamp;
  uint32_t packets;
  uint32_t octets;
};
// RTCP Receiver Report (RR) header
struct rtcp_rr_header {
  uint8_t vprc;
  uint8_t pt; // = 201 (RR)
  uint16_t length;
  uint32_t ssrc;
  struct {
    uint32_t ssrc;
    uint32_t losses; // fraction_lost << 24 | cum#lost
    uint32_t hiseq;
    uint32_t jitter;
    uint32_t lsr;
    uint32_t dlsr;
  } reports[];
};

#endif
