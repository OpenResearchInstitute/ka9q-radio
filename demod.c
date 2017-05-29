// $Id: demod.c,v 1.11 2017/05/29 10:29:28 karn Exp karn $
// Common demod thread for all modes
// Takes commands from UDP packets on a socket
#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <unistd.h>
#include <limits.h>
#include <math.h>
#include <complex.h>
#undef I

#include "radio.h"


int process_command(char *cmdbuf,int len);

extern int Ctl_port;

int In_count = 0;
float dc_alpha = 0.00001; // high pass filter coefficient for offset and I/Q imbalance estimates
float phi_alpha = 0.000001;
float power_alpha = 0.0002; // high pass filter coefficient for power estimates

void demod(void);

const float SCALE = 1./32768.;

int Bufnum;
const int nbuffers = 4;
complex float Buffers[4][4096]; // FIX THIS



void proc_samples(short *sp,int cnt){

  // Channel gain balance coefficients
  float gain_i=1,gain_q=1,pscale = 0;
  if(Demod.power_i != 0 && Demod.power_q != 0){
    gain_q = sqrt((Demod.power_i + Demod.power_q)/(2*Demod.power_q));
    gain_i = sqrt((Demod.power_i + Demod.power_q)/(2*Demod.power_i));
    pscale = 1/(Demod.power_i + Demod.power_q);
  }
  float secphi,tanphi;
  secphi = 1/sqrt(1 - Demod.sinphi * Demod.sinphi);
  tanphi = Demod.sinphi * secphi;


  int i;
  for(i=0;i<cnt;i++){
    float samp_i,samp_q;
    // Remove and update DC offsets
    samp_i = sp[2*i] - Demod.DC_i;    samp_q = sp[2*i+1] - Demod.DC_q;
    Demod.DC_i += dc_alpha * samp_i;     Demod.DC_q += dc_alpha * samp_q;
    // Unity peak amplitude
    samp_i *= SCALE;                  samp_q *= SCALE;
    // Update channel power estimates
    Demod.power_i += power_alpha * (samp_i * samp_i - Demod.power_i);
    Demod.power_q += power_alpha * (samp_q * samp_q - Demod.power_q);    
    // Balance gains, keeping constant total energy
    samp_i *= gain_i;                       samp_q *= gain_q;
    // Correct phase
    samp_q = samp_q * secphi - tanphi*samp_i;
    // Update residual phase error estimate
    Demod.sinphi += phi_alpha * (samp_i * samp_q) * pscale;
    // Pass corrected sample to demodulator filter, invoke when full
    Buffers[Bufnum][In_count++] = CMPLXF(samp_i,samp_q);
    if(In_count == 4096){ // FIX THIS
      extern int Demod_sock;
      complex float *x;
      
      x = &Buffers[Bufnum][0];
      write(Demod_sock,&x,sizeof(x));
      In_count = 0;
      Bufnum++;
      if(Bufnum == nbuffers)
	Bufnum = 0;
    }
  }
}
