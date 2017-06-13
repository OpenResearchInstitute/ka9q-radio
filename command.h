#ifndef _COMMAND_H
#define _COMMAND_H 1

#include <stdint.h>
#include <sys/socket.h>
enum cmd {
  SENDSTAT=1,
  SETSTATE,
};

enum mode {
  AM = 1,
  CAM,     // coherent AM
  IQ,
  ISB,
  USB,
  CWU,
  LSB,
  CWL,
  NFM,
  FM,
  WFM,
};

struct command {
  enum cmd cmd;
  enum mode mode;
  double first_LO;
  double second_LO;
  double second_LO_rate;
  double calibrate;
};
struct modetab {
  enum mode mode;
  char name[10];
  float dial;       // Display frequency offset (mainly for CW/RTTY)
  float tunestep;   // Default tuning step
  float low;        // Lower edge of IF passband
  float high;       // Upper edge of IF passband
  float gain;       // Gain of filter
};
// Sent in each RTP packet right after header
// NB! because we just copy this into the network stream, it's important that the compiler
// not add any extra padding. To avoid this, the size must be a multiple of 8, the size of the double
struct status {
  double frequency;
  uint32_t samprate;
  uint8_t lna_gain;
  uint8_t mixer_gain;
  uint8_t if_gain;
  uint8_t unused; // pad to 16 bytes
};


extern struct modetab Modes[];
extern const int Nmodes;

extern int Rtpsock;
extern struct sockaddr_storage Input_source_address;
extern socklen_t Rtpaddrlen;

extern struct sockaddr_storage Input_mcast_sockaddr;  



#endif
