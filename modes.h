#ifndef _MODES_H
#define _MODES_H 1

enum demod_type {
  LINEAR_DEMOD = 0,     // Linear demodulation, i.e., everything else: SSB, CW, DSB, CAM, IQ
  FM_DEMOD,             // Frequency demodulation
};

// Internal format of entries in /usr/local/share/ka9q-radio/modes.txt
struct modetab {
  char name[16];
  enum demod_type demod_type;
  int pll;
  int square;
  int channels;     // 1 or 2
  int isb;
  int flat;
  int env;
  double shift;      // Audio frequency shift, Hz (mainly for CW/RTTY)
  float tunestep;   // Default tuning step
  float low;        // Lower edge of IF passband, Hz
  float high;       // Upper edge of IF passband, Hz
  float attack_rate; // dB/sec
  float recovery_rate; // dB/sec
  float hangtime;    // sec
  float headroom;    // dB
};

extern struct modetab Modes[];
extern int Nmodes;

// Load mode definition table
int readmodes(char *);
char *demod_name(enum demod_type type);

#endif
