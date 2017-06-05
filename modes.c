// $Id: modes.c,v 1.5 2017/06/05 06:09:16 karn Exp karn $
#include "command.h"

struct modetab Modes[] = {
  {0,     "",     0,       0,       0,    0, },
  {AM,   "AM",    0,    1000,   -5000, 5000, }, // Envelope detected AM
  {CAM, "CAM",    0,    1000,   -5000, 5000, }, // Coherent AM  
  {IQ,   "IQ",    0,      10,   -5000, 5000, },
  {ISB, "ISB",    0,     100,   -5000, 5000, },
  {USB, "USB",    0,     100,     300, 3000, },
  {CWU, "CWU",  400,      10,     200, 600, },
  {LSB, "LSB",    0,     100,   -3000, -300, },
  {CWL, "CWL", -400,      10,   -600, -200, },
  {NFM, "NFM",    0,    1000,   -3000, 3000, }, // For D*star
  {FM,   "FM",    0,    1000,   -8000, 8000, },
  //  {FM,   "FM",    0,    1000,   -10000, 10000, },
  //  {WFM, "WFM",    0,  100000,       0,    0, }  // No pre-detection filter
};
const int Nmodes = sizeof(Modes)/sizeof(struct modetab) - 1;
