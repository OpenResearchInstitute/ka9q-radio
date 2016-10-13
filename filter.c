// $Id$
// General purpose filter package using fast convolution (overlap-save)
// and the FFTW3 FFT package
// Generates transfer functions using Kaiser window
// Optional output decimation by integer factor
// Complex input and transfer functions, complex or real output
#include <complex.h>
#include <math.h>
#include <fftw3.h>
#include <malloc.h>
#include <memory.h>
#include <assert.h>
#include "dsp.h"
#include "filter.h"

// When decimation is used, we assume the filter response drops to negligible
// well below the decimated (lower) Nyquist rate so we can avoid the extra work of adding in
// the aliased frequency components needed to produce exactly the same result as
// decimating the time-domain output by the same ratio

// Real filer is faster type that uses c2r IFFTs to discard imaginary component of output
// Useful for SSB and VSB
// NB: Input and response are still both full-size complex

// response[] must be SIMD-aligned (e.g., with fftw_alloc) and will be freed by delete_filter()
struct filter *create_filter(int L,int M, complex float *response,int decimate,enum type type){
  struct filter *f;
  const int N = L + M - 1;
  const int N_dec = N / decimate;

  f = calloc(1,sizeof(*f));
  f->type = type;
  f->blocksize_in = L;
  f->impulse_length = M;
  f->decimate = decimate;

  // Parameter sanity check
  if((N % decimate) != 0)
    fprintf(stderr,"Warning: FFT size %'u is not divisible by decimation ratio %d\n",N,decimate);

  if((M - 1) % decimate != 0)
    fprintf(stderr,"Warning: Filter length %'u - 1 is not divisible by decimation ratio %d\n",M,decimate);

  f->input_buffer = fftwf_alloc_complex(N);
  memset(f->input_buffer,0,(M-1)*sizeof(*f->input_buffer)); // Clear earlier state
  f->input = f->input_buffer + M - 1;
  f->response = response;
  // If response is null, only allocate the input buffer. Used for, e.g., wideband FM
  if(response != NULL){
    f->fdomain = fftwf_alloc_complex(N);
    f->blocksize_out = f->blocksize_in / decimate;
    f->fwd_plan = fftwf_plan_dft_1d(N,f->input_buffer,f->fdomain,FFTW_FORWARD,FFTW_ESTIMATE);
    if(type == REAL){
      f->output_buffer.r = fftwf_alloc_real(N_dec);
      f->output.r = f->output_buffer.r + (M - 1)/decimate;
      f->rev_plan = fftwf_plan_dft_c2r_1d(N_dec,f->fdomain,f->output_buffer.r,FFTW_ESTIMATE);
    } else {
      f->output_buffer.c = fftwf_alloc_complex(N_dec);  
      f->output.c = f->output_buffer.c + (M - 1)/decimate;
      f->rev_plan = fftwf_plan_dft_1d(N_dec,f->fdomain,f->output_buffer.c,FFTW_BACKWARD,FFTW_ESTIMATE);
    }
  }
  return f;
}

int execute_filter(struct filter *f){
  int N;
  int N_dec;
  int i,n;
  
  if(f == NULL || f->type == NONE || f->response == NULL)
    return -1;
  N = f->blocksize_in + f->impulse_length - 1;
  N_dec = N / f->decimate;
  fftwf_execute(f->fwd_plan);  // Forward transform
  memmove(f->input_buffer,f->input_buffer + f->blocksize_in,(f->impulse_length - 1)*sizeof(*f->input_buffer)); // Save for next

  f->fdomain[0] *= f->response[0];      // DC
  if(f->type == COMPLEX){ // Actually the simplest
    for(i=N-1,n=1; n < N_dec/2; n++,i--){
      f->fdomain[n] *= f->response[n];    // Positive frequency
      f->fdomain[N_dec-n] = f->fdomain[i] * f->response[i]; // Negative frequency
    }
  } else if(f->type == CROSS_CONJ){
    // Hack for ISB; forces negative frequencies onto I, positive onto Q
    for(i=N-1,n=1; n < N_dec/2; n++,i--){
      complex t; // Needed when decimate == 1
      t = f->response[i] * (f->fdomain[i] - conjf(f->fdomain[n]));
      f->fdomain[n] = f->response[n] * (f->fdomain[n] + conjf(f->fdomain[i])); // positive
      f->fdomain[N_dec-n] = t; // negative
    }
  } else if(f->type == REAL){
    for(i=N-1,n=1; n < N_dec/2; n++,i--)
      f->fdomain[n] = f->fdomain[n] * f->response[n] + conjf(f->fdomain[i] * f->response[i]);
  }
  f->fdomain[N_dec/2] *= f->response[N_dec/2]; // Nyquist frequency
  fftwf_execute(f->rev_plan); // Note: c2r version destroys fdomain[]
  return 0;
}

int delete_filter(struct filter *f){
  fftwf_destroy_plan(f->fwd_plan);
  fftwf_destroy_plan(f->rev_plan);  
  fftwf_free(f->input_buffer);
  fftwf_free(f->output_buffer.c);
  fftwf_free(f->response);
  fftwf_free(f->fdomain);
  free(f);
  return 0;
}

// Window shape factor for Kaiser window
// Transition region is approx sqrt(1+Beta^2)
double Kaiser_beta = 3.0;


// Hamming window
const double hamming(int n,int M){
  const double alpha = 25./46;
  const double beta = (1-alpha);

  return alpha - beta * (cos(2*M_PI*n/(M-1)));
}

// Hann / "Hanning" window
const double hann(int n,int M){
    return 0.5 - 0.5 * (cos(2*M_PI*n/(M-1)));
}

// Exact Blackman window
const double blackman(int n,int M){
  const double a0 = 7938./18608;
  const double a1 = 9240./18608;
  const double a2 = 1430./18608;
  return a0 - a1*cos(2*M_PI*n/(M-1)) + a2*cos(4*M_PI*n/(M-1));
}

// Modified Bessel function of the 0th kind, used by the Kaiser window
static const double i0(double x){
  double sum = 0;
  double t,term;
  int k;

  t = 0.25 * x * x;
  sum = 1 + t;
  term = t;
  for(k=2;k<40;k++){
    term *= t/(k*k);
    sum += term;
    if(term < 1e-12 * sum)
      break;
  }
  return sum;
}

// Jim Kaiser was in my Bellcore department in the 1980s. Wonder whatever happened to him.
const double kaiser(int n,int M, double beta){
  double p = 2.0*n/(M-1) - 1;
  return i0(M_PI*beta*sqrt(1-p*p)) / i0(M_PI*beta);
}

// Apply Kaiser window to filter frequency response
// "response" is SIMD-aligned array of N complex floats
// Impulse response will be limited to first M samples in the time domain
// Phase is adjusted so "time zero" (center of impulse response) is at M/2
int window_filter(int L,int M,complex float *response,double beta){
  fftwf_plan fwd_filter_plan,rev_filter_plan;
  int N,n;
  complex float *buffer;

  N = L + M - 1;
  // fftw_plan can overwrite its buffers, so we're forced to make a temp. Ugh.
  buffer = fftwf_alloc_complex(N);
  fwd_filter_plan = fftwf_plan_dft_1d(N,buffer,buffer,FFTW_FORWARD,FFTW_ESTIMATE);
  rev_filter_plan = fftwf_plan_dft_1d(N,buffer,buffer,FFTW_BACKWARD,FFTW_ESTIMATE);

  // Convert to time domain
  memcpy(buffer,response,N*(sizeof *buffer));
  fftwf_execute(rev_filter_plan);
  fftwf_destroy_plan(rev_filter_plan);

  // Shift to beginning of buffer, apply window and scale (N*N)
  for(n = M - 1; n >= 0; n--)
    buffer[n] = buffer[(n-M/2+N)%N] * kaiser(n,M,beta)/(N*N);

  // Pad with zeroes on right side
  memset(buffer+M,0,(N-M)*sizeof(*buffer));

#if 0
  fprintf(stderr,"Filter impulse response, shifted, windowed and zero padded\n");
  for(n=0;n< N;n++)
    fprintf(stderr,"%d %lg %lg\n",n,crealf(buffer[n]),cimagf(buffer[n]));
#endif
  
  // Now back to frequency domain
  fftwf_execute(fwd_filter_plan);
  fftwf_destroy_plan(fwd_filter_plan);

#if 0
  fprintf(stderr,"Filter response amplitude\n");
  for(n=0;n<N;n++){
    float f = n*192000./N;
    fprintf(stderr,"%.1f %.1f\n",f,power2dB(cnrmf(buffer[n])));
  }
  fprintf(stderr,"\n");
#endif
  memcpy(response,buffer,N*(sizeof *response));
  fftwf_free(buffer);
  return 0;
}
// Real-only counterpart to window_filter()
// response[] is only N/2+1 elements containing DC and positive frequencies only
// Negative frequencies are inplicitly the conjugate of the positive frequencies
int window_rfilter(int L,int M,complex float *response,double beta){
  complex float *buffer;
  float *timebuf;
  fftwf_plan fwd_filter_plan,rev_filter_plan;
  int N,n;

  N = L + M - 1;
  buffer = fftwf_alloc_complex(N/2 + 1); // plan destroys its input
  timebuf = fftwf_alloc_real(N);
  rev_filter_plan = fftwf_plan_dft_c2r_1d(N,buffer,timebuf,FFTW_ESTIMATE);
  fwd_filter_plan = fftwf_plan_dft_r2c_1d(N,timebuf,buffer,FFTW_ESTIMATE);
  
  // Convert to time domain
  memcpy(buffer,response,(N/2+1)*sizeof(*buffer));
  fftwf_execute(rev_filter_plan);
  fftwf_destroy_plan(rev_filter_plan);

  // Shift to beginning of buffer, apply window and scale (N*N)
  for(n = M - 1; n >= 0; n--)
    timebuf[n] = timebuf[(n-M/2+N)%N] * kaiser(n,M,beta)/(N*N);
  
  // Pad with zeroes on right side
  memset(timebuf+M,0,(N-M)*sizeof(*timebuf));
#if 0
  printf("Filter impulse response, shifted, windowed and zero padded\n");
  for(n=0;n< M;n++)
    printf("%d %lg\n",n,timebuf[n]);
#endif
  
  // Now back to frequency domain
  fftwf_execute(fwd_filter_plan);
  fftwf_destroy_plan(fwd_filter_plan);
  fftwf_free(timebuf);
#if 0
  printf("Filter frequency response\n");
  for(n=0; n < N/2 + 1; n++)
    printf("%d %g %g (%.1f dB)\n",n,crealf(buffer[n]),cimagf(buffer[n]),
	   power2dB(cnrm(buffer[n])));
#endif
  memcpy(response,buffer,(N/2+1)*sizeof(*response));
  fftwf_free(buffer);
  return 0;
}
