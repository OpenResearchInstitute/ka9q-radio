// $Id: fm.h,v 1.2 2016/10/13 23:31:14 karn Exp karn $
#ifndef _FM_H
#define _FM_H 1

#include "radio.h"

float fm_snr(complex float *,int);
int do_fm(float *output,complex float *buffer,int L,complex float *state);
#endif
