// $Id: radio.c,v 1.49 2017/08/04 14:55:17 karn Exp karn $
// Lower part of radio program - control LOs, set frequency/mode, etc
#define _GNU_SOURCE 1
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>
#undef I
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>


#include "audio.h"
#include "radio.h"
#include "filter.h"
#include "dsp.h"


// Preferred A/D sample rate; ignored by funcube but may be used by others someday
const int ADC_samprate = 192000;


// Completely fill buffer from corrected I/O input queue
// Block until enough data is available
int fillbuf(struct demod *demod,complex float *buffer,int cnt){
  // This assumes cnt <= DATASIZE, otherwise it will deadlock!
  // The mutex protects demod->write_ptr
  pthread_mutex_lock(&demod->data_mutex);
  while((DATASIZE + demod->write_ptr - demod->read_ptr) % DATASIZE < cnt)
    pthread_cond_wait(&demod->data_cond,&demod->data_mutex);
  pthread_mutex_unlock(&demod->data_mutex);  

  // Copy in contiguous chunks from circular buffer
  int i = cnt;
  while(i > 0){
    int chunk = DATASIZE - demod->read_ptr;
    chunk = min(chunk,i);    // Largest contiguous segment of buffer before wraparound
    memcpy(buffer,&demod->corr_data[demod->read_ptr],chunk*sizeof(complex float));
    demod->read_ptr = (demod->read_ptr + chunk) % DATASIZE;
    i -= chunk;
    buffer += chunk;
  }
  return cnt;
}


// Get true first LO frequency
const double get_first_LO(const struct demod *demod){
  assert(demod != NULL);
  assert(!isnan(demod->status.frequency));
  assert(!isnan(demod->calibrate));
	 
  return demod->status.frequency * (1 + demod->calibrate);  // True frequency, as adjusted
}


// Return current frequency of carrier frequency at current first IF
const double get_second_LO(const struct demod *demod){
  assert(demod != NULL);
  assert(!isnan(demod->second_LO));

  return demod->second_LO;

}

// Set frequency with optional front end tuning
double set_freq(struct demod *demod,double f,int force){
  assert(demod != NULL);
  assert(!isnan(f));


  // Wait for sample rate to be known
  pthread_mutex_lock(&demod->status_mutex);
  while(demod->status.samprate == 0)
    pthread_cond_wait(&demod->status_cond,&demod->status_mutex);
  double lo1 = get_first_LO(demod);
  pthread_mutex_unlock(&demod->status_mutex);
  
  double new_lo2 = -(f - lo1 - demod->dial_offset);
  double new_lo1;
  // If the new LO2 is out of range, or if we're forced, recenter LO2
  // and retune LO1
  if(force || !LO2_in_range(demod,new_lo2,1)){
    // Assume we'll keep tuning in the same direction
    if(new_lo2 < 0)
      new_lo2 = demod->status.samprate/4.;
    else 
      new_lo2 = -(demod->status.samprate/4.);
    new_lo1 = f + new_lo2 - demod->dial_offset;
    
    // returns actual frequency, which may be a fraction of a Hz
    // different from requested because of calibration offset and
    // the fact that the tuner can only tune in 1 Hz steps
    lo1 = set_first_LO(demod,new_lo1,force);
    new_lo2 += (lo1 - new_lo1);
  }
  // If front end doesn't retune don't retune LO2 either (e.g., recording)
  if(LO2_in_range(demod,new_lo2,1))
     set_second_LO(demod,new_lo2);
  return f;
}

// Return current frequency, including dial offset
const double get_freq(const struct demod *demod){
  assert(demod != NULL);
  return get_first_LO(demod) - get_second_LO(demod) + demod->dial_offset;
}

// Set tuner LO
// Note: single precision floating point is not accurate enough at VHF and above
// demod->first_LO isn't updated here, but by the
// incoming status frames so it don't change right away
double set_first_LO(struct demod *demod,double first_LO,int force){
  assert(demod != NULL);
  assert(!isnan(first_LO));
  if(!force && first_LO == get_first_LO(demod))
    return first_LO;

  if(first_LO > 0){
    // Set tuner to integer nearest requested frequency after decalibration
    demod->requested_status.frequency = round(first_LO / (1 + demod->calibrate)); // What we send to the tuner
    // These need a way to set
    demod->requested_status.samprate = ADC_samprate; // Preferred samprate; ignored by funcube
    demod->requested_status.lna_gain = 1;
    demod->requested_status.mixer_gain = 1;
    demod->requested_status.if_gain = 0;
    
    // Wait up to 1 sec for the frequency to actually change
    // We need to limit this to keep two or more receivers from continually fighting over it, or if it's a recording
    int i;
    struct timespec ts;
    struct timeval tv;
    gettimeofday(&tv,NULL);
    ts.tv_sec = tv.tv_sec + 1; // Wait 1 sec
    ts.tv_nsec = 1000 * tv.tv_usec;

    pthread_mutex_lock(&demod->status_mutex);
    for(i=0;i<10;i++){
      if(demod->requested_status.frequency == demod->status.frequency)
	break;

      if((demod->input_source_address.sa_family == AF_INET || demod->input_source_address.sa_family == AF_INET6)
	 && sendto(demod->ctl_fd,&demod->requested_status,sizeof(demod->requested_status),0,(struct sockaddr *)&demod->input_source_address,sizeof(demod->input_source_address)) == -1)
	perror("sendto control socket");
      if(pthread_cond_timedwait(&demod->status_cond,&demod->status_mutex,&ts) == -1)
	break;
    }
    pthread_mutex_unlock(&demod->status_mutex);
    assert(i < 50);
  }
  return demod->status.frequency * (1 + demod->calibrate);
}

// If avoid_alias is true, return 1 if specified carrier frequency is in range of LO2 given
// sampling rate, filter setting and alias region
//
// If avoid_alias is false, simply test that specified frequency is between +/- samplerate/2
int LO2_in_range(struct demod *demod,double f,int avoid_alias){
  assert(demod != NULL);

  // Wait until the sample rate is known
  pthread_mutex_lock(&demod->status_mutex);
  while(demod->samprate == 0)
    pthread_cond_wait(&demod->status_cond,&demod->status_mutex);
  pthread_mutex_unlock(&demod->status_mutex);
    
  if(avoid_alias)
    return f >= demod->min_IF + max(0,demod->high)
	    && f <= demod->max_IF + min(0,demod->low);
  else
    return f >= -demod->samprate/2 && f <= demod->samprate/2;

}

// Set second local oscillator (the one in software)
// Only limit range to +/- samprate/2; the caller must avoid the alias region, e.g., with LO2_in_range()
double set_second_LO(struct demod *demod,double second_LO){
  assert(demod != NULL);
  assert(!isnan(second_LO));

  // Wait until the sample rate is known
  pthread_mutex_lock(&demod->status_mutex);
  while(demod->samprate == 0)
    pthread_cond_wait(&demod->status_cond,&demod->status_mutex);
  pthread_mutex_unlock(&demod->status_mutex);
    
  assert(second_LO <= demod->samprate/2 && second_LO >= -demod->samprate/2);
  if(isnan(creal(demod->second_LO_phase)) || isnan(cimag(demod->second_LO_phase)) || cnrm(demod->second_LO_phase) < 0.999)
    demod->second_LO_phase = 1;

  // When setting frequencies, assume TCXO also drives sample clock, so use same calibration
  // In case sample rate isn't set yet, just remember the frequency but don't divide by zero
  demod->second_LO_phase_step = csincos(2*M_PI*second_LO/demod->samprate);
  return demod->second_LO = second_LO;
}

int set_mode(struct demod *demod,enum mode mode,int defaults){
  assert(demod != NULL);

  // Send EOF to current demod thread, if any, to cause clean exit
  demod->terminate = 1;
  pthread_join(demod->demod_thread,NULL); // Wait for it to finish
  demod->terminate = 0;

  demod->mode = mode;
  if(defaults || isnan(demod->dial_offset))
    demod->dial_offset = Modes[mode].dial;
  if(defaults || isnan(demod->low))
    demod->low = Modes[mode].low;
  if(defaults || isnan(demod->high))
    demod->high = Modes[mode].high;
  // Suppress these in display unless they're used
  demod->snr = NAN;
  demod->foffset = NAN;
  demod->pdeviation = NAN;
  demod->cphase = NAN;
  demod->plfreq = NAN;

  // Wait until we know the sample rate
  pthread_mutex_lock(&demod->status_mutex);
  while(demod->samprate == 0)
    pthread_cond_wait(&demod->status_cond,&demod->status_mutex);
  pthread_mutex_unlock(&demod->status_mutex);

  double lo2 = get_second_LO(demod);
  // Might now be out of range because of change in filter passband
  if(!LO2_in_range(demod,lo2,1))
    set_freq(demod,get_freq(demod),1);

  switch(mode){
  case DSB:
    pthread_create(&demod->demod_thread,NULL,demod_dsb,demod);
    break;
  case NFM:
  case FM:
    pthread_create(&demod->demod_thread,NULL,demod_fm,demod);
    break;
  case USB:
  case LSB:
  case CWU:
  case CWL:
    pthread_create(&demod->demod_thread,NULL,demod_ssb,demod);
    break;
  case CAM:
    pthread_create(&demod->demod_thread,NULL,demod_cam,demod);
    break;
  case AM:
    pthread_create(&demod->demod_thread,NULL,demod_am,demod);
    break;
  case IQ:
  case ISB:
    pthread_create(&demod->demod_thread,NULL,demod_iq,demod);
    break;
  case WFM:
  case NO_MODE:
    break;
  }
  return 0;
}      

int set_filter(struct demod *demod,float low,float high){
  assert(demod != NULL);
  assert(demod->filter != NULL);
  struct filter * const filter = demod->filter;
  
  float const dsamprate = demod->samprate / filter->decimate; // Decimated (output) sample rate
  int const L_dec = filter->olen;
  int const M_dec = (filter->impulse_length - 1) / filter->decimate + 1;
  int const N_dec = L_dec + M_dec - 1;
  int const N = filter->ilen + filter->impulse_length - 1;

  if(high > demod->max_IF || low < demod->min_IF || high <= low)
    return -1;

  float gain = 1./((float)N);
#if 0
  if(filter->out_type == REAL || filter->out_type == CROSS_CONJ)
    gain *= M_SQRT1_2;
#endif

  complex float * const response = fftwf_alloc_complex(N_dec);
  int n;
  for(n=0;n<N_dec;n++){
    float f;
    if(n <= N_dec/2)
      f = (float)n * dsamprate / N_dec;
    else
      f = (float)(n-N_dec) * dsamprate / N_dec;
    if(f >= low && f <= high)
      response[n] = gain;
    else
      response[n] = 0;
  }
  window_filter(L_dec,M_dec,response,demod->kaiser_beta);

  // Hot swap with existing response, if any, using mutual exclusion
  pthread_mutex_lock(&filter->mutex);
  complex float *tmp = filter->response;
  filter->response = response;
  pthread_mutex_unlock(&filter->mutex);
  fftwf_free(tmp);
  demod->low = low;
  demod->high = high;
  return 0;
}



// Set TXCO calibration for front end
// + means clock is fast, - means clock is slow
int set_cal(struct demod *demod,double cal){
  assert(demod != NULL);

  double f = get_freq(demod);
  demod->calibrate = cal;
  demod->samprate = demod->status.samprate * (1 + cal);
  set_freq(demod,f,0);
  return 0;
}
int spindown(struct demod *demod,complex float *data,int len){
  assert(demod != NULL);
  assert(data != NULL);
  assert(!isnan(creal(demod->second_LO_phase_step)) && !isnan(cimag(demod->second_LO_phase_step)));
  assert(cnrm(demod->second_LO_phase) != 0);

  // Apply 2nd LO
  int n;
  for(n=0; n < len; n++){
    assert(!isnan(crealf(data[n])) && !isnan(cimagf(data[n])));
    data[n] *= demod->second_LO_phase;
    demod->second_LO_phase *= demod->second_LO_phase_step;
  }
  // Renormalize to guard against accumulated roundoff error
  demod->second_LO_phase /= cabs(demod->second_LO_phase);
  return 0;
}
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
    demod->corr_data[(demod->write_ptr + i) % DATASIZE] = samp;
  }
  // Update estimates of DC offset, signal powers and phase error
  demod->DC_i += DC_alpha * (samp_i_sum - cnt * demod->DC_i);
  demod->DC_q += DC_alpha * (samp_q_sum - cnt * demod->DC_q);
  demod->power_i += Power_alpha * (samp_i_sq_sum - cnt * demod->power_i);
  demod->power_q += Power_alpha * (samp_q_sq_sum - cnt * demod->power_q);

  float dpn = 2 * dotprod / (samp_i_sq_sum + samp_q_sq_sum);
  demod->sinphi += Power_alpha * cnt * (dpn - demod->sinphi);

  pthread_mutex_lock(&demod->data_mutex);
  demod->write_ptr += cnt;
  demod->write_ptr %= 65536;
  pthread_cond_broadcast(&demod->data_cond);
  pthread_mutex_unlock(&demod->data_mutex);
}
