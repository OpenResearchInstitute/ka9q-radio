#ifndef _COMMAND_H
#define _COMMAND_H 1
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
  double dial;       // Display frequency offset (mainly for CW/RTTY)
  double tunestep;   // Default tuning step
  double low;        // Lower edge of IF passband
  double high;       // Upper edge of IF passband
  int channels;      // Number of audio channels
};
extern struct modetab Modes[];
extern const int Nmodes;


#endif
