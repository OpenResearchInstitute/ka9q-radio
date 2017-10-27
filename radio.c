// $Id: radio.c,v 1.76 2017/10/22 07:37:26 karn Exp karn $
// Lower part of radio program - control LOs, set frequency/mode, etc
#define _GNU_SOURCE 1
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <stdint.h>
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
  if(demod == NULL)
    return;

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
    if(demod->interpolate == 1){
      // Not interpolating; directly copy large blocks
      chunk = min(chunk,DATASIZE - demod->read_ptr); // How much can we copy contiguously?
      chunk = min(chunk,i); // How much do we need?
      
      memcpy(buffer,&demod->corr_data[demod->read_ptr],chunk*sizeof(complex float));
      demod->read_ptr = (demod->read_ptr + chunk) % DATASIZE;
      i -= chunk;
      buffer += chunk;
    } else {
      // copy only one sample at a time, then stuff zeroes
      *buffer++ = demod->corr_data[demod->read_ptr++];
      demod->read_ptr %= DATASIZE;
      for(int j=1; j<demod->interpolate; j++){
	*buffer++ = 0;
      }
      i -= demod->interpolate;
    }
  }
  return cnt;
}


// The funcube dongle uses the Mirics MSi001 tuner. It has a fractional N synthesizer that can't actually do integer frequency steps.
// This formula is hacked down from code from Howard Long; it's what he uses in the firmware so I can figure out
// the *actual* frequency. Of course, we still have to correct it for the TCXO offset.

// This needs to be made modular since other tuners will be completely different!

double fcd_actual(unsigned int u32Freq){
  typedef unsigned int UINT32;
  typedef unsigned long long UINT64;

  const UINT32 u32Thresh=3250U;
  const UINT32 u32FRef=26000000U;
  double f64FAct;
  
  struct
  {
    UINT32 u32Freq;
    UINT32 u32FreqOff;
    UINT32 u32LODiv;
  } *pts,ats[]=
      {
	{4000000U,130000000U,16U},
	{8000000U,130000000U,16U},
	{16000000U,130000000U,16U},
	{32000000U,130000000U,16U},
	{75000000U,130000000U,16U},
	{125000000U,0U,32U},
	{142000000U,0U,16U},
	{148000000U,0U,16U},
	{300000000U,0U,16U},
	{430000000U,0U,4U},
	{440000000U,0U,4U},
	{875000000U,0U,4U},
	{UINT32_MAX,0U,2U},
	{0U,0U,0U}
      };
  for(pts = ats; u32Freq>=pts->u32Freq; pts++)
    ;

  if (pts->u32Freq == 0)
    pts--;
      
  UINT64 u64FSynth=(((UINT64)u32Freq)+pts->u32FreqOff)*pts->u32LODiv;
  UINT32 u32Int=(UINT32)(u64FSynth/(u32FRef*4));
  UINT32 u32Frac4096=(UINT32)((((u64FSynth<<12)*u32Thresh/(u32FRef*4))-(u32Int<<12)*u32Thresh));
  UINT32 u32Frac=u32Frac4096>>12;
  UINT32 u32AFC=u32Frac4096-(u32Frac<<12);
      
  f64FAct = (4.0 * u32FRef / (double)pts->u32LODiv) * (u32Int + ((u32Frac * 4096.0 + u32AFC) / (u32Thresh * 4096.))) - pts->u32FreqOff;
  
  // double f64step = ( (4.0 * u32FRef) / (pts->u32LODiv * (double)u32Thresh) ) / 4096.0;
  
  //      printf("f64step = %'lf, u32LODiv = %'u, u32Frac = %'d, u32AFC = %'d, u32Int = %'d, u32Thresh = %'d, u32FreqOff = %'d, f64FAct = %'lf err = %'lf\n",
  //	     f64step, pts->u32LODiv, u32Frac, u32AFC, u32Int, u32Thresh, pts->u32FreqOff,f64FAct,f64FAct - u32Freq);
  return f64FAct;
}



// Get true first LO frequency
double const get_first_LO(const struct demod * const demod){
  if(demod == NULL)
    return NAN;
	 
  return fcd_actual(demod->status.frequency) * (1 + demod->calibrate);  // True frequency, as adjusted;

}


double get_second_LO(struct demod * const demod){
  if(demod == NULL)
    return NAN;
  pthread_mutex_lock(&demod->second_LO_mutex);
  double f = demod->second_LO;
  pthread_mutex_unlock(&demod->second_LO_mutex);  
  return f;
}

double get_freq(struct demod * const demod){
  if(demod == NULL)
    return NAN;
  return get_first_LO(demod) - get_second_LO(demod);
}

int set_doppler(struct demod * const demod,double freq,double rate){
  pthread_mutex_lock(&demod->doppler_mutex);
  demod->doppler = freq;
  demod->doppler_rate = rate;
  demod->doppler_phasor_step = csincos(-2*M_PI*freq / demod->samprate);
  demod->doppler_phasor_step_step = csincos(-2*M_PI*rate / (demod->samprate*demod->samprate));
  if(!is_phasor_init(demod->doppler_phasor)) // Initialized?
    demod->doppler_phasor = 1;
  pthread_mutex_unlock(&demod->doppler_mutex);
  return 0;
}
double get_doppler(struct demod * const demod){
  pthread_mutex_lock(&demod->doppler_mutex);
  double f = demod->doppler;
  pthread_mutex_unlock(&demod->doppler_mutex);  
  return f;
}
double get_doppler_rate(struct demod * const demod){
  pthread_mutex_lock(&demod->doppler_mutex);
  double f = demod->doppler_rate;
  pthread_mutex_unlock(&demod->doppler_mutex);  
  return f;
}




// Set radio frequency with optional IF selection
// new_lo2 is explicitly allowed to be NAN. If it is, that's a "don't care"
// we'll try to pick a new LO2 that avoids retuning LO1,
// and if that isn't possible we'll pick a default:
// (usually +/- 48 kHz, samprate/4)
double set_freq(struct demod * const demod,double const f,double new_lo2){
  if(demod == NULL)
    return NAN;

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
  return get_freq(demod);
}

// Preferred A/D sample rate; ignored by funcube but may be used by others someday
const int ADC_samprate = 192000;

// Set tuner LO
// Note: single precision floating point is not accurate enough at VHF and above
// demod->first_LO isn't updated here, but by the
// incoming status frames so it don't change right away
double set_first_LO(struct demod * const demod,double const first_LO){
  if(demod == NULL)
    return NAN;
  if(first_LO == get_first_LO(demod))
    return first_LO;

  if(demod->tuner_lock){
    // Just return actual frequency, don't touch anything
    return get_first_LO(demod);
  }

  if(first_LO > 0){
    // Set tuner to integer nearest requested frequency after decalibration
    demod->requested_status.frequency = round(first_LO / (1 + demod->calibrate)); // What we send to the tuner
    // These need a way to set
    demod->requested_status.samprate = ADC_samprate; // Preferred samprate; ignored by funcube
    demod->requested_status.lna_gain = 0xff;    // 0xff means "don't change"
    demod->requested_status.mixer_gain = 0xff;
    demod->requested_status.if_gain = 0xff;
    
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
	sdraddr.sin_port = htons(ntohs(sdraddr.sin_port)+1);
	if(sendto(demod->ctl_fd,&demod->requested_status,sizeof(demod->requested_status),0,(struct sockaddr *)&sdraddr,sizeof(sdraddr)) == -1)
	  perror("sendto control socket");
	if(pthread_cond_timedwait(&demod->status_cond,&demod->status_mutex,&ts) == -1)
	  break;
      }
    }
    pthread_mutex_unlock(&demod->status_mutex);
  }
  return get_first_LO(demod);
}

// If avoid_alias is true, return 1 if specified carrier frequency is in range of LO2 given
// sampling rate, filter setting and alias region
//
// If avoid_alias is false, simply test that specified frequency is between +/- samplerate/2
int LO2_in_range(struct demod * const demod,double const f,int const avoid_alias){
  if(demod == NULL)
    return -1;

  // Wait until the sample rate is known
  pthread_mutex_lock(&demod->status_mutex);
  while(demod->samprate == 0)
    pthread_cond_wait(&demod->status_cond,&demod->status_mutex);
  pthread_mutex_unlock(&demod->status_mutex);
    
  if(avoid_alias)
    return f >= demod->min_IF + max(0,demod->high)
	    && f <= demod->max_IF + min(0,demod->low);
  else {
    return fabs(f) <=  0.5 * demod->samprate; // within Nyquist limit?
  }
}

// The next two frequency setting functions depend on the sample rate
// If it's not known (0) don't try to set the phasor step; let the input routine call us again

// Set second local oscillator (the one in software)
// the caller must avoid aliasing, e.g., with LO2_in_range()
double set_second_LO(struct demod * const demod,double const second_LO){
  if(demod == NULL)
    return NAN;

  if(!is_phasor_init(demod->second_LO_phasor)) // Initialized?
    demod->second_LO_phasor = 1;

  // When setting frequencies, assume TCXO also drives sample clock, so use same calibration
  // In case sample rate isn't set yet, just remember the frequency but don't divide by zero
  if(demod->samprate != 0)
    demod->second_LO_phasor_step = csincos(2*M_PI*second_LO/demod->samprate);

  demod->second_LO = second_LO;
  return second_LO;
}

// Set audio shift after downconversion and detection (linear modes only: SSB, IQ, DSB)
double set_shift(struct demod * const demod,double const shift){
  demod->shift = shift;
  if(demod->samprate != 0)
    demod->shift_phasor_step = csincos(2*M_PI*shift*demod->decimate/demod->samprate);
  if(!is_phasor_init(demod->shift_phasor))
    demod->shift_phasor = 1;
  return shift;
}


int set_mode(struct demod * const demod,const char * const mode,int const defaults){
  if(demod == NULL)
    return -1;

  int mindex;
  for(mindex=0; mindex<Nmodes; mindex++){
    if(strcasecmp(mode,Modes[mindex].name) == 0)
      break;
  }
  if(mindex == Nmodes)
    return -1; // Unregistered mode
  // Current mode is calibrate, save
  if(demod->flags & CAL){
    // Save calibration file
    savecal(demod);
  }

  // Kill current demod thread, if any, to cause clean exit
  demod->terminate = 1;
  pthread_join(demod->demod_thread,NULL); // Wait for it to finish
  demod->terminate = 0;

  // if the mode argument points to demod->mode, avoid the copy; can cause an abort
  if(demod->mode != mode)
    strlcpy(demod->mode,mode,sizeof(demod->mode));

  strlcpy(demod->demod_name, Modes[mindex].demod_name, sizeof(demod->demod_name));


  if(defaults || isnan(demod->low) || isnan(demod->high)){
    demod->low = Modes[mindex].low;
    demod->high = Modes[mindex].high;
  }

  // Ensure low < high
  if(demod->high < demod->low){
    float const tmp = demod->low;
    demod->low = demod->high;
    demod->high = tmp;
  }
  demod->flags = Modes[mindex].flags;
  demod->attack_rate = Modes[mindex].attack_rate;
  demod->recovery_rate = Modes[mindex].recovery_rate;
  demod->hangtime = Modes[mindex].hangtime;
  
  // Suppress these in display unless they're used
  demod->snr = NAN;
  demod->pdeviation = NAN;
  demod->cphase = NAN;
  demod->plfreq = NAN;
  demod->spare = NAN;

  // Wait until we know the sample rate
  pthread_mutex_lock(&demod->status_mutex);
  while(demod->samprate == 0)
    pthread_cond_wait(&demod->status_cond,&demod->status_mutex);
  pthread_mutex_unlock(&demod->status_mutex);

  if(defaults || isnan(demod->shift)){
    if(demod->shift != Modes[mindex].shift){
      // Adjust tuning for change in frequency shift
      set_freq(demod,get_freq(demod) + Modes[mindex].shift - demod->shift,NAN);
    }
    set_shift(demod,Modes[mindex].shift);
  }

  // Load calibration file
  loadcal(demod);


  double lo2 = demod->second_LO;
  // Might now be out of range because of change in filter passband
  if(!LO2_in_range(demod,lo2,1))
    set_freq(demod,get_freq(demod),NAN);

  pthread_create(&demod->demod_thread,NULL,Modes[mindex].demod,demod);
  return 0;
}      


// Set TXCO calibration for front end
// + means clock is fast, - means clock is slow
int set_cal(struct demod * const demod,double const cal){
  if(demod == NULL)
    return -1;

  double f = get_freq(demod);
  demod->calibrate = cal;
  // Don't get deadlocked if this is before we know the sample rate
  // e.g., with the -c command line option
  if(demod->status.samprate != 0){
    demod->samprate = demod->status.samprate * (1 + cal);
    set_freq(demod,f,NAN); // Keep original dial frequency
  }
  return 0;
}
// Apply LO2 to input samples
// Length of data input obtained from demod->filter->i_len
int spindown(struct demod * const demod,complex float const * const data){
  if(demod == NULL || data == NULL || demod->filter == NULL)
    return -1;

  struct filter * const filter = demod->filter;

  pthread_mutex_lock(&demod->second_LO_mutex);
  if(is_phasor_init(demod->second_LO_phasor)) { // Initialized?
    // Apply 2nd LO, compute average pre-filter signal power
    for(int n=0; n < filter->ilen; n++){
      filter->input.c[n] = data[n] * demod->second_LO_phasor;
      demod->second_LO_phasor *= demod->second_LO_phasor_step;
    }
    demod->second_LO_phasor /= cabs(demod->second_LO_phasor);
  }
  pthread_mutex_unlock(&demod->second_LO_mutex);
  
  // Apply Doppler, if active
  pthread_mutex_lock(&demod->doppler_mutex);
  if(is_phasor_init(demod->doppler_phasor)){ // Initialized?
    for(int n=0; n < filter->ilen; n++){
      filter->input.c[n] *= demod->doppler_phasor;
      demod->doppler_phasor *= demod->doppler_phasor_step;
      demod->doppler_phasor_step *= demod->doppler_phasor_step_step;
    }
    demod->doppler_phasor /= cabs(demod->doppler_phasor);
    demod->doppler_phasor_step /= cabs(demod->doppler_phasor_step);
  }
  pthread_mutex_unlock(&demod->doppler_mutex);
  return 0;
}

// Compute noise spectral density - experimental, my algorithm
// The problem is telling signal from noise
// Heuristic: first average all bins outside the bandwidth
// Then recompute the average, tossing bins > 3 dB above the previous average
// Hopefully this will get rid of any signals from the noise estimate
float const compute_n0(struct demod const * const demod){
  if(demod == NULL || demod->filter == NULL)
    return NAN;
  
  struct filter const *f = demod->filter;
  int const N = f->ilen + f->impulse_length - 1;
  float power_spectrum[N];
  
  // Compute smoothed power spectrum
  // There will be some spectral leakage because the convolution FFT we're using is unwindowed
  for(int n=0;n<N;n++){
    power_spectrum[n] = cnrmf(f->fdomain[n]);
  }  

  // compute average energy outside passband, then iterate computing a new average that
  // omits bins > 3dB above the previous average. This should pick up only the noise
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
