#include "command.h"

struct modetab Modes[] = {
  {0,     "",     0,       0,       0,    0, 0,},
  {AM,   "AM",    0,    1000,   -5000, 5000, 1,}, // Envelope detected AM
  {CAM, "CAM",    0,    1000,   -5000, 5000, 1,}, // Coherent AM  
  {IQ,   "IQ",    0,      10,   -5000, 5000, 2,},
  {ISB, "ISB",    0,     100,   -5000, 5000, 2,},
  {USB, "USB",    0,     100,     300, 3000, 1,},
  {CWU, "CWU",  750,      10,     500, 1000, 1,},
  {LSB, "LSB",    0,     100,   -3000, -300, 1,},
  {CWL, "CWL", -750,      10,   -1000, -500, 1,},
  {NFM, "NFM",    0,    1000,   -3000, 3000, 1,}, // For D*star
  {FM,   "FM",    0,    1000,   -7500, 7500, 1,},
  //  {WFM, "WFM",    0,  100000,       0,    0, 1,}  // No pre-detection filter
};
const int Nmodes = sizeof(Modes)/sizeof(struct modetab) - 1;
