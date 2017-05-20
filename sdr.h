// $Id: sdr.h,v 1.3 2016/10/14 04:35:50 karn Exp karn $
#ifndef _SDR_H
#define _SDR_H 1

#include <complex.h>
#undef I

int mirics_gain(double f,int gr,char *bb, char *lna,char *mix);
int front_end_init(int,int,int);
int get_adc(short *,int);
void closedown(int);

#endif
