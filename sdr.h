// $Id: sdr.h,v 1.2 2016/10/13 23:59:21 karn Exp karn $
#ifndef _SDR_H
#define _SDR_H 1

#include <complex.h>
#undef I

int mirics_gain(double f,int gr,int *bb, int *lna,int *mix);
int front_end_init(int,int,int);
float get_adc(complex float *,int);
void closedown(int);

#endif
