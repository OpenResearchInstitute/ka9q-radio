// $Id: demod.c,v 1.25 2017/07/26 11:23:57 karn Exp karn $
// Common I/Q processing for all modes
#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <unistd.h>
#include <limits.h>
#include <math.h>
#include <complex.h>
#undef I

#include "radio.h"
#include "filter.h"


float const DC_alpha = 0.00001;    // high pass filter coefficient for DC offset estimates, per sample
float const Power_alpha = 0.00001; // high pass filter coefficient for power and I/Q imbalance estimates, per sample
float const SCALE = 1./SHRT_MAX;   // Scale signed 16-bit int to float in range -1, +1

// Preprocessing of samples performed for all demodulators
// Remove DC biases, equalize I/Q power, correct phase imbalance
// Update power measurement
void proc_samples(struct demod *demod,const int16_t *sp,int cnt){
  // gain and phase balance coefficients
  float gain_i=1, gain_q=1, secphi=1, tanphi=0;
  if(demod->power_i != 0 && demod->power_q != 0){ // Avoid floating point exceptions at startup
    float const totpower = demod->power_i + demod->power_q;
    gain_q = sqrtf(totpower/(2*demod->power_q));         // Power ratio to amplitude ratio requires sqrt()
    gain_i = sqrtf(totpower/(2*demod->power_i));
    secphi = 1/sqrtf(1 - demod->sinphi * demod->sinphi); // sec(phi) = 1/cos(phi)
    tanphi = demod->sinphi * secphi;                     // tan(phi) = sin(phi) * sec(phi) = sin(phi)/cos(phi)
  }
  int i;
  complex float buffer[cnt];
  float samp_i_sum = 0, samp_q_sum = 0;        // sums of I and Q, for DC offset
  float samp_i_sq_sum = 0, samp_q_sq_sum = 0;  // sums of I^2 and Q^2, for power and gain balance
  float dotprod = 0;                           // sum of I*Q, for phase balance

  for(i=0;i<cnt;i++){
    // Remove and update DC offsets
    float samp_i = *sp++ * SCALE;
    samp_i_sum += samp_i;
    samp_i -= demod->DC_i;
    samp_i_sq_sum += samp_i * samp_i;

    float samp_q = *sp++ * SCALE;
    samp_q_sum += samp_q;
    samp_q -= demod->DC_q;
    samp_q_sq_sum += samp_q * samp_q;

    // Balance gains, keeping constant total energy
    samp_i *= gain_i;                  samp_q *= gain_q;

    dotprod += samp_i * samp_q;
    // Correct phase
    samp_q = secphi * samp_q - tanphi * samp_i;
    assert(!isnan(samp_q) && !isnan(samp_i));
    complex float samp = CMPLXF(samp_i,samp_q);
    // Experimental notch filter
    if(demod->nf)
      samp = notch(demod->nf,samp);

    // Final corrected sample
    buffer[i] = samp;
  }
  // Update estimates of DC offset, signal powers and phase error
  demod->DC_i += DC_alpha * (samp_i_sum - cnt * demod->DC_i);
  demod->DC_q += DC_alpha * (samp_q_sum - cnt * demod->DC_q);
  demod->power_i += Power_alpha * (samp_i_sq_sum - cnt * demod->power_i);
  demod->power_q += Power_alpha * (samp_q_sq_sum - cnt * demod->power_q);

  float dpn = 2 * dotprod / (samp_i_sq_sum + samp_q_sq_sum);
  demod->sinphi += Power_alpha * cnt * (dpn - demod->sinphi);

  // Pass to demodulator thread (ssb/fm/iq etc)
  write(demod->corr_iq_write_fd,buffer,sizeof(buffer));
}
