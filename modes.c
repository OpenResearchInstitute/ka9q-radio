// $Id: modes.c,v 1.8 2017/07/02 04:29:55 karn Exp karn $
#include "radio.h"

struct modetab Modes[] = {
  {AM,  "AM",     0,    1000,   -5000, 5000, }, // Envelope detected AM
  {CAM, "CAM",    0,    1000,   -5000, 5000, }, // Coherent AM  
  {IQ,  "IQ",     0,      10,   -5000, 5000, }, // Stereo: I on left, Q on right
  {ISB, "ISB",    0,     100,   -5000, 5000, }, // Independent sideband: LSB on left, USB on right
  {USB, "USB",    0,     100,     300, 3000, }, // Upper sideband
  {CWU, "CWU",  400,      10,     200,  600, }, // Upper sideband, CW filter
  {LSB, "LSB",    0,     100,   -3000, -300, }, // Lower sideband
  {CWL, "CWL", -400,      10,    -600, -200, }, // Lower sideband, CW filter
  {NFM, "NFM",    0,    1000,   -3000, 3000, }, // For D*star
  {FM,  "FM",     0,    1000,   -8000, 8000, }, // Ordinary standard NBFM
  {DSB, "DSB",    0,      10,   -5000, 5000, }, // Double sideband AM, suppressed carrier
  {WFM, "WFM",    0,  100000,  -96000,96000, },

};
const int Nmodes = sizeof(Modes)/sizeof(struct modetab);
