// $Id: radio.c,v 1.62 2017/09/07 17:56:37 karn Exp karn $
// Lower part of radio program - control LOs, set frequency/mode, etc
#define _GNU_SOURCE 1
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#if defined(linux)
#include <bsd/string.h>
#endif
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

// Lower half of input thread
// Preprocessing of samples performed for all demodulators
// Remove DC biases, equalize I/Q power, correct phase imbalance
// Update power measurement

float const DC_alpha = 0.00001;    // high pass filter coefficient for DC offset estimates, per sample
float const Power_alpha = 0.00001; // high pass filter coefficient for power and I/Q imbalance estimates, per sample
float const SCALE = 1./SHRT_MAX;   // Scale signed 16-bit int to float in range -1, +1

void proc_samples(struct demod * const demod,int16_t const *sp,int const cnt){
  // gain and phase balance coefficients
  assert(demod != NULL);
  assert(demod->imbalance > 0);
  assert(demod->sinphi >= -1 && demod->sinphi <= 1);

  float const gain_q = sqrtf(0.5 * (1 + demod->imbalance));
  float const gain_i = sqrtf(0.5 * (1 + 1./demod->imbalance));
  float const secphi = 1/sqrtf(1 - demod->sinphi * demod->sinphi); // sec(phi) = 1/cos(phi)
  float const tanphi = demod->sinphi * secphi;                     // tan(phi) = sin(phi) * sec(phi) = sin(phi)/cos(phi)


  float samp_i_sum = 0, samp_q_sum = 0;        // sums of I and Q, for DC offset
  float samp_i_sq_sum = 0, samp_q_sq_sum = 0;  // sums of I^2 and Q^2, for power and gain balance
  float dotprod = 0;                           // sum of I*Q, for phase balance
  for(int i=0;i<cnt;i++){
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
  demod->imbalance += Power_alpha * cnt * ((samp_i_sq_sum / samp_q_sq_sum) - demod->imbalance);

  float dpn = 2 * dotprod / (samp_i_sq_sum + samp_q_sq_sum);
  demod->sinphi += Power_alpha * cnt * (dpn - demod->sinphi);

  pthread_mutex_lock(&demod->data_mutex);
  demod->write_ptr += cnt;
  demod->write_ptr %= DATASIZE;
  pthread_cond_broadcast(&demod->data_cond);
  pthread_mutex_unlock(&demod->data_mutex);
}


// Completely fill buffer from corrected I/O input queue
// Block until enough data is available
int fillbuf(struct demod * const demod,complex float *buffer,int const cnt){
  for(int i = cnt;i > 0;){
    // The mutex protects demod->write_ptr
    pthread_mutex_lock(&demod->data_mutex);
    while(demod->write_ptr == demod->read_ptr)
      pthread_cond_wait(&demod->data_cond,&demod->data_mutex);
    
    int chunk = (demod->write_ptr - demod->read_ptr + DATASIZE) % DATASIZE; // How much is available?
    pthread_mutex_unlock(&demod->data_mutex);  // Done looking at write_ptr
    chunk = min(chunk,DATASIZE - demod->read_ptr); // How much can we copy contiguously?
    chunk = min(chunk,i); // How much do we need?
		
    memcpy(buffer,&demod->corr_data[demod->read_ptr],chunk*sizeof(complex float));
    demod->read_ptr = (demod->read_ptr + chunk) % DATASIZE;
    i -= chunk;
    buffer += chunk;
  }
  return cnt;
}


// Get true first LO frequency
double const get_first_LO(const struct demod * const demod){
  assert(demod != NULL);
  assert(!isnan(demod->status.frequency));
  assert(!isnan(demod->calibrate));
	 
  return demod->status.frequency * (1 + demod->calibrate);  // True frequency, as adjusted
}


// Set radio frequency with optional IF selection
// new_lo2 is explicitly allowed to be NAN. If it is, that's a "don't care"
// we'll try to pick a new LO2 that avoids retuning LO1,
// and if that isn't possible we'll pick a default:
// (usually +/- 48 kHz, samprate/4)
double set_freq(struct demod * const demod,double const f,double new_lo2){
  assert(demod != NULL);
  assert(!isnan(f));

  // Wait for sample rate to be known
  pthread_mutex_lock(&demod->status_mutex);
  while(demod->status.samprate == 0)
    pthread_cond_wait(&demod->status_cond,&demod->status_mutex);
  pthread_mutex_unlock(&demod->status_mutex);

  if(isnan(new_lo2) || !LO2_in_range(demod,new_lo2,0)){
    new_lo2 = -(f - get_first_LO(demod));

    // If the required new LO2 is out of range, recenter LO2 and retune LO1
    if(!LO2_in_range(demod,new_lo2,1)){
      // Assume we'll keep tuning in the same direction
#if 0
      if(new_lo2 < 0)
	new_lo2 = demod->status.samprate/4.;
      else 
	new_lo2 = -(demod->status.samprate/4.);
#else
      // Experimentally fix IF to -48 kHz to check calibration offsets
      new_lo2 = demod->status.samprate/4.;
#endif    
    }
  }
  double const new_lo1 = f + new_lo2;
    
  // returns actual frequency, which may be a fraction of a Hz
  // different from requested because of calibration offset and
  // the fact that the tuner can only tune in 1 Hz steps
  double const actual_lo1 = set_first_LO(demod,new_lo1);
  new_lo2 += (actual_lo1 - new_lo1); // fold the difference into LO2

  // If front end doesn't retune don't retune LO2 either (e.g., recording)
  if(LO2_in_range(demod,new_lo2,0))
     set_second_LO(demod,new_lo2);

  // Set to new actual frequency, rather than one requested
  demod->frequency = get_first_LO(demod) - demod->second_LO;
  return demod->frequency;
}

// Preferred A/D sample rate; ignored by funcube but may be used by others someday
const int ADC_samprate = 192000;

// Set tuner LO
// Note: single precision floating point is not accurate enough at VHF and above
// demod->first_LO isn't updated here, but by the
// incoming status frames so it don't change right away
double set_first_LO(struct demod * const demod,double const first_LO){
  assert(demod != NULL);
  assert(!isnan(first_LO));
  if(first_LO == get_first_LO(demod))
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
    // Tries are limited to keep two or more receivers from continually fighting, or if it's a recording
    struct timespec ts;
    struct timeval tv;
    gettimeofday(&tv,NULL);
    ts.tv_sec = tv.tv_sec + 1; // Wait 1 sec
    ts.tv_nsec = 1000 * tv.tv_usec;

    pthread_mutex_lock(&demod->status_mutex);
    for(int i=0;i<10;i++){
      if(demod->requested_status.frequency == demod->status.frequency)
	break;

      if(demod->input_source_address.sa_family == AF_INET){
	// If we know the sender, send it a tuning request
	struct sockaddr_in sdraddr;
	memcpy(&sdraddr,&demod->input_source_address,sizeof(sdraddr));
	sdraddr.sin_port = htons(CTLPORT);
	if(sendto(demod->ctl_fd,&demod->requested_status,sizeof(demod->requested_status),0,(struct sockaddr *)&sdraddr,sizeof(sdraddr)) == -1)
	  perror("sendto control socket");
	if(pthread_cond_timedwait(&demod->status_cond,&demod->status_mutex,&ts) == -1)
	  break;
      }
    }
    pthread_mutex_unlock(&demod->status_mutex);
  }
  return demod->status.frequency * (1 + demod->calibrate);
}

// If avoid_alias is true, return 1 if specified carrier frequency is in range of LO2 given
// sampling rate, filter setting and alias region
//
// If avoid_alias is false, simply test that specified frequency is between +/- samplerate/2
int LO2_in_range(struct demod * const demod,double const f,int const avoid_alias){
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
double set_second_LO(struct demod * const demod,double const second_LO){
  assert(demod != NULL);
  assert(!isnan(second_LO));

  // Wait until the sample rate is known
  pthread_mutex_lock(&demod->status_mutex);
  while(demod->samprate == 0)
    pthread_cond_wait(&demod->status_cond,&demod->status_mutex);
  pthread_mutex_unlock(&demod->status_mutex);
    
  assert(second_LO <= demod->samprate/2 && second_LO >= -demod->samprate/2);
  if(isnan(creal(demod->second_LO_phasor)) || isnan(cimag(demod->second_LO_phasor)) || cnrm(demod->second_LO_phasor) < 0.999)
    demod->second_LO_phasor = 1;

  // When setting frequencies, assume TCXO also drives sample clock, so use same calibration
  // In case sample rate isn't set yet, just remember the frequency but don't divide by zero
  demod->second_LO_phasor_step = csincos(2*M_PI*second_LO/demod->samprate);
  demod->second_LO = second_LO;
  return second_LO;
}

// Set audio shift after downconversion and detection (linear modes only: SSB, IQ, DSB)
double set_shift(struct demod * const demod,double const shift){
  demod->shift = shift;
  demod->shift_phasor_step = csincos(2*M_PI*shift*demod->decimate/demod->samprate);
  if(cabs(demod->shift_phasor) == 0)
    demod->shift_phasor = 1;
  return shift;
}


int set_mode(struct demod * const demod,const char * const mode,int const defaults){
  assert(demod != NULL);

  int mindex;
  for(mindex=0; mindex<Nmodes; mindex++){
    if(strcasecmp(mode,Modes[mindex].name) == 0)
      break;
  }
  if(mindex == Nmodes)
    return -1; // Unregistered mode
  // Kill current demod thread, if any, to cause clean exit
  demod->terminate = 1;
  pthread_join(demod->demod_thread,NULL); // Wait for it to finish
  demod->terminate = 0;

  // if the mode argument points to demod->mode, avoid the copy; can cause an abort
  if(demod->mode != mode)
    strlcpy(demod->mode,mode,sizeof(demod->mode));

  if(defaults || isnan(demod->shift))
    demod->shift = Modes[mindex].shift;
  if(defaults || isnan(demod->low))
    demod->low = Modes[mindex].low;
  if(defaults || isnan(demod->high))
    demod->high = Modes[mindex].high;
  // Ensure low < high
  if(demod->high < demod->low){
    float const tmp = demod->low;
    demod->low = demod->high;
    demod->high = tmp;
  }
  demod->flags = Modes[mindex].flags;
  if(cabs(demod->shift_phasor) == 0)
    demod->shift_phasor = 1;

  if(cabs(demod->shift_phasor_step) == 0)
    demod->shift_phasor_step = 1;
  
  // Suppress these in display unless they're used
  demod->snr = NAN;
  demod->pdeviation = NAN;
  demod->cphase = NAN;
  demod->plfreq = NAN;

  // Wait until we know the sample rate
  pthread_mutex_lock(&demod->status_mutex);
  while(demod->samprate == 0)
    pthread_cond_wait(&demod->status_cond,&demod->status_mutex);
  pthread_mutex_unlock(&demod->status_mutex);

  double lo2 = demod->second_LO;
  // Might now be out of range because of change in filter passband
  if(!LO2_in_range(demod,lo2,1))
   set_freq(demod,demod->frequency,NAN);

  pthread_create(&demod->demod_thread,NULL,Modes[mindex].demod,demod);
  return 0;
}      


// Set TXCO calibration for front end
// + means clock is fast, - means clock is slow
int set_cal(struct demod * const demod,double const cal){
  assert(demod != NULL);
  assert(!isnan(cal));

  demod->calibrate = cal;
  // Don't get deadlocked if this is before we know the sample rate
  // e.g., with the -c command line option
  if(demod->status.samprate != 0){
    demod->samprate = demod->status.samprate * (1 + cal);
    set_freq(demod,demod->frequency,NAN); // Keep original dial frequency
  }
  return 0;
}
// Apply LO2 to input samples, placing the resulting IF in the demod's filter input buffer
// This function no longer updates demod->second_LO phasor directly
// It returns the updated phasor, and you must store it back into demod->send_LO_phasor yourself
// This simplifies demodulators that do repeated spindowns (but don't forget filter state)
// Length of data input obtained from demod->filter->i_len
complex float spindown(struct demod * const demod,complex float const * const data){
  assert(demod != NULL);
  if(demod->second_LO == 0)
    return 0; // Probably not set yet, but in any event nothing to do

  assert(demod->filter != NULL);
  struct filter  * const filter = demod->filter;
  assert(data != NULL);
  assert(!isnan(creal(demod->second_LO_phasor_step)) && !isnan(cimag(demod->second_LO_phasor_step)));
  assert(cnrm(demod->second_LO_phasor) != 0);

  // Apply 2nd LO, compute average pre-filter signal power

  complex float second_LO_phasor = demod->second_LO_phasor;
  for(int n=0; n < filter->ilen; n++){
    assert(!isnan(crealf(data[n])) && !isnan(cimagf(data[n])));
    filter->input.c[n] = data[n] * second_LO_phasor;
    second_LO_phasor *= demod->second_LO_phasor_step;
  }
  // Renormalize to guard against accumulated roundoff error
  return second_LO_phasor / cabs(second_LO_phasor);
}

// Compute noise spectral density - experimental, my algorithm
// The problem is telling signal from noise
// Heuristic: first average all bins outside the bandwidth
// Then recompute the average, tossing bins > 3 dB above the previous average
// Hopefully this will get rid of any signals from the noise estimate
float const compute_n0(struct demod const * const demod){
  assert(demod != NULL  && demod->filter != NULL);
  struct filter const *f = demod->filter;
  int const N = f->ilen + f->impulse_length - 1;
  float power_spectrum[N];
  
  // Compute smoothed power spectrum
  // There will be some spectral leakage because the convolution FFT we're using is unwindowed
  for(int n=0;n<N;n++){
    power_spectrum[n] = cnrmf(f->fdomain[n]);
  }  

  // compute average energy outside passband, then iterate computing a new average that
  // omits N > 3dB above the previous average.

  float avg_n = INFINITY;
  for(int iter=0;iter<2;iter++){
    int noisebins = 0;
    float new_avg_n = 0;
    for(int n=0;n<N;n++){
      float f;
      if(n <= N/2)
	f = (float)(n * demod->samprate) / N;
      else
	f = (float)((n-N) * demod->samprate) / N;
      
      if(f >= demod->low && f <= demod->high)
	continue; // Avoid passband
      
      float const s = power_spectrum[n];
      if(s < avg_n * 2){ // +3dB threshold
	new_avg_n += s;
	noisebins++;
      }
    }
    new_avg_n /= noisebins;
    avg_n = new_avg_n;
  }
  // return noise power per Hz
  return avg_n / (N*demod->samprate);
}
