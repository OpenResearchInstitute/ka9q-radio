// $Id$
#ifndef _FM_H
#define _FM_H 1

#include "radio.h"

float squelch_fm(complex float *,int);
int do_fm(float *output,complex float *buffer,int L,complex float *state);
#endif
