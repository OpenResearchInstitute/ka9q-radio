#include <math.h>
#include <stddef.h>
#include <assert.h>
// Window shape factor for Kaiser window
// Transition region is approx sqrt(1+Beta^2)
float Kaiser_beta = 3.0;

// Modified Bessel function of the 0th kind, used by the Kaiser window
static const float i0(float const x){
  const float t = 0.25 * x * x;
  float sum = 1 + t;
  float term = t;
  for(int k=2;k<40;k++){
    term *= t/(k*k);
    sum += term;
    if(term < 1e-12 * sum)
      break;
  }
  return sum;
}


#if 0 // Available if you ever want them

// Hamming window
const static float hamming(int const n,int const M){
  const float alpha = 25./46;
  const float beta = (1-alpha);

  return alpha - beta * cos(2*M_PI*n/(M-1));
}

// Hann / "Hanning" window
const static float hann(int const n,int const M){
    return 0.5 - 0.5 * cos(2*M_PI*n/(M-1));
}

// Exact Blackman window
const static float blackman(int const n,int const M){
  float const a0 = 7938./18608;
  float const a1 = 9240./18608;
  float const a2 = 1430./18608;
  return a0 - a1*cos(2*M_PI*n/(M-1)) + a2*cos(4*M_PI*n/(M-1));
}

// Jim Kaiser was in my Bellcore department in the 1980s. Wonder whatever happened to him.
// Superseded by make_kaiser() routine that more efficiently computes entire window at once
static float const kaiser(int const n,int const M, float const beta){
  static float old_beta = NAN;
  static float old_inv_denom;

  // Cache old value of beta, since it rarely changes
  if(beta != old_beta){
    old_beta = beta;
    old_inv_denom = 1. / i0(M_PI*beta);
  }
  const float p = 2.0*n/(M-1) - 1;
  return i0(M_PI*beta*sqrtf(1-p*p)) * old_inv_denom;
}
#endif

// Compute an entire Kaiser window
// More efficient than repeatedly calling kaiser(n,M,beta)
int make_kaiser(float * const window,unsigned int const M,float const beta){
  assert(window != NULL);
  if(window == NULL)
    return -1;
  // Precompute unchanging partial values
  float const numc = M_PI * beta;
  float const inv_denom = 1. / i0(numc); // Inverse of denominator
  float const pc = 2.0 / (M-1);

  // The window is symmetrical, so compute only half of it and mirror
  // this won't compute the middle value in an odd-length sequence
  for(int n = 0; n < M/2; n++){
    float const p = pc * n  - 1;
    window[M-1-n] = window[n] = i0(numc * sqrtf(1-p*p)) * inv_denom;
  }
  // If sequence length is odd, middle value is unity
  if(M & 1)
    window[(M-1)/2] = 1; // The -1 is actually unnecessary

  return 0;
}

#if 0
// Empirical formula for Kaiser beta parameter vs stoppband attenuation in dB
static float beta(float a){
  if(a > 50)
    return 0.1102*(a - 8.7);
  if(a >= 21)
    return 0.5842*pow((a - 21),0.4) + 0.07886*(a - 21);
  return 0;
}

// Filter order as function of stoppband attenuation in dB and width of transition region (normalized?)
static int fir_order(float a, float delta){
  return (a - 7.95) / (2.285 * delta);
}
#endif
