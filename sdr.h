// $Id$
#ifndef _SDR_H
#define _SDR_H 1

#include <complex.h>
#undef I

int front_end_init(int,int,int);
float get_adc(complex float *,int);
void closedown(int);

#endif
