// $Id: demod.c,v 1.9 2017/05/25 05:10:56 karn Exp karn $
// Common demod thread for all modes
// Takes commands from UDP packets on a socket
#define _GNU_SOURCE 1 // allow bind/connect/recvfrom without casting sockaddr_in6
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#undef I
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "dsp.h"
#include "filter.h"
#include "radio.h"
#include "fm.h"
#include "audio.h"
#include "command.h"


int process_command(char *cmdbuf,int len);
int process_am();
int process_cam();
int process_iq();
int process_isb();
int process_ssb();
int process_fm();
int process_wfm();

extern int Ctl_port;

int In_count = 0;
float dc_alpha = 0.00001; // high pass filter coefficient for offset and I/Q imbalance estimates
float phi_alpha = 0.000001;
float power_alpha = 0.0002; // high pass filter coefficient for power estimates

void demod(void);

const float SCALE = 1./32768.;

void proc_samples(short *sp,int cnt){
  if(Demod.filter == NULL || Demod.filter->input == NULL){
    return; // We're not ready; drop
  }

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
    Demod.filter->input[In_count++] = CMPLXF(samp_i,samp_q);
    if(In_count == Demod.filter->blocksize_in){
      demod();
      In_count = 0;
    }
  }
}


void demod(){

  spindown(Demod.filter->input,Demod.filter->blocksize_in); // 2nd LO
  
  int i;
  i = execute_filter(Demod.filter);
  assert(i == 0);
  switch(Demod.mode){
   case AM:
    process_am();
    break;
  case CAM:
    process_cam();
    break;
  case IQ:
  case ISB: // Processed the same except for filter hack
    process_iq();
    break;
  case USB:
  case CWU:
  case LSB:
  case CWL:
    process_ssb();
    break;
  case NFM:
  case FM:
    process_fm();
    break;
  case WFM:
    process_wfm();
    break;
  }
}


int process_ssb(){
  // Automatic gain control
  Demod.amplitude = amplitude(Demod.filter->output.r,Demod.filter->blocksize_out);
  float snn = Demod.amplitude / Demod.noise; // (S+N)/N amplitude ratio
  Demod.snr = (snn*snn) -1; // S/N as power ratio
  ssb_agc();
  
  put_mono_audio(Demod.filter->output.r,Demod.filter->blocksize_out,Demod.gain);
  return 0;
}

// Envelope detection: take magnitude of complex samples, ignoring phase
// Also find average carrier level for AGC and DC removal
int process_am(){
  float average;
  int n;
  float audio[Demod.filter->blocksize_out];

  average = 0;
  for(n=0; n < Demod.filter->blocksize_out; n++)
    average += audio[n] = cabs(Demod.filter->output.c[n]);
  average /= Demod.filter->blocksize_out;
  Demod.amplitude = average;
  
  // AM AGC is carrier-driven
  Demod.gain = Headroom / average;
  for(n=0; n<Demod.filter->blocksize_out; n++)
    audio[n] -= average; // Subtract carrier to remove DC
  
  put_mono_audio(audio,Demod.filter->blocksize_out,Demod.gain); // we do our own
  return 0;
}
// Experimental semi-coherent AM detection
int process_cam(){
    complex float phase;
    int n;
    float audio[Demod.filter->blocksize_out];
    static complex float lastphase;
    double freqerror;
    
    phase = 0;
    for(n=0; n < Demod.filter->blocksize_out; n++)
      phase += Demod.filter->output.c[n];

    phase = conj(phase) / cabs(phase);

    // Rotate signal onto I axis, measure DC (carrier) level
    Demod.amplitude = 0;
    for(n=0; n < Demod.filter->blocksize_out; n++)
      Demod.amplitude += audio[n] = creal(Demod.filter->output.c[n] * phase);
    
    Demod.amplitude /= Demod.filter->blocksize_out;

    // Remove carrier DC
    for(n=0; n < Demod.filter->blocksize_out; n++)
      audio[n] -= Demod.amplitude;

    // Frequency error is the phase of this block minus the last, times blocks/sec
    // Phase was already flipped, hence the minus
    // Only move a fraction of the error at one time
    freqerror = -0.01 * carg(phase * conj(lastphase))/(2*M_PI) * Demod.samprate/Demod.filter->blocksize_in;
    lastphase = phase;
    set_second_LO(-freqerror + Demod.second_LO,0);

    // AM AGC is carrier-driven
    Demod.gain = Headroom / Demod.amplitude;
    put_mono_audio(audio,Demod.filter->blocksize_out,Demod.gain); // we do our own
    return 0;
}

int process_iq(){
  // Find average amplitude for AGC
  Demod.amplitude = camplitude(Demod.filter->output.c,Demod.filter->blocksize_out);
  ssb_agc();
  
  put_stereo_audio(Demod.filter->output.c,Demod.filter->blocksize_out,Demod.gain);
  return 0;
}

int process_fm(){
  float audio[Demod.filter->blocksize_out];

  Demod.snr = fm_snr(Demod.filter->output.c,Demod.filter->blocksize_out);

  // If squelch is closed, just let the output drain
  if(Demod.snr > 2){
    do_fm(audio,Demod.filter->output.c,Demod.filter->blocksize_out,&Demod.fmstate);
    put_mono_audio(audio,Demod.filter->blocksize_out,Demod.gain);
  }
  return 0;
}
int process_wfm(){
  float audio[Demod.filter->blocksize_in];

  // If squelch is closed, just let the output drain
  // We don't use a pre-demod filter, so we work with the *INPUT* buffer only
  if((Demod.snr = fm_snr(Demod.filter->input,Demod.filter->blocksize_in)) < 2)
    return 0;

  do_fm(audio,Demod.filter->input,Demod.filter->blocksize_in,&Demod.fmstate);
  put_mono_audio(audio,Demod.filter->blocksize_in,Demod.gain);
  return 0;
}
