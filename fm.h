// $Id: fm.h,v 1.3 2017/05/20 01:15:07 karn Exp $
#ifndef _FM_H
#define _FM_H 1

#include "radio.h"

float fm_snr(complex float *,int);
int do_fm(float *output,complex float *buffer,int L,complex float *state);
#endif
