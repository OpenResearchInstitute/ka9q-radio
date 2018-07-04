// half-band filter for sample rate decimation by powers of 2
#include <string.h>
#include <assert.h>
#include "decimate.h"

// Pick up vectorized versions if available
#if defined(__SSSE3__)
  #include <x86intrin.h>  // GCC-compatible compiler, targeting x86/x86-64

/* Folded half-band 15-tap filter
   Only four non-unity coefficents are needed
   Note ordering of coefficients: coeff[0] is at the tails, not the center

  |--   even[3]   even[2]    even[1]    even[0]  <-- in first
  v
drop

  --     odd[3]    odd[2]     odd[1]     odd[0]  <-- in second
  |
  |       +         +          +          +
  |
  |-->oldodd[3]  oldodd[2]  oldodd[1]  oldodd[0] --------|
          *          *           *          *            v
       coeff[3]   coeff[2]   coeff[1]   coeff[0]        drop
          v          v           v          v
        temp[3]    temp[2]    temp[1]    temp[0]
       sum(temp[3...0] + even[3]) to give output
 */


// x86 vectorized version: SSSE3 or better for horizontal add
#define shiftleft(arg,n) _mm_castsi128_ps(_mm_bslli_si128(_mm_castps_si128(arg),4*n))
#define shiftright(arg,n) _mm_castsi128_ps(_mm_bsrli_si128(_mm_castps_si128(arg),4*n))

void hb15_block(struct hb15_state *state,float *output,float *input,int cnt){
  __m128 coeffs;
  __m128 even_samples;
  __m128 odd_samples;
  __m128 old_odd_samples;

  coeffs = _mm_loadu_ps(state->coeffs);  // coeffs in xmm5
  even_samples = _mm_loadu_ps(state->even_samples); // xmm3
  odd_samples = _mm_loadu_ps(state->odd_samples); // xmm1
  old_odd_samples = _mm_loadu_ps(state->old_odd_samples); // xmm2

  while(cnt--){
    __m128 temp;

    temp = _mm_load_ss(input++); // xmm0[0] = *input++
    even_samples = _mm_move_ss(even_samples,temp);  // even_samples[0] = temp[0]

    temp = _mm_load_ss(input++);
    odd_samples = _mm_move_ss(odd_samples,temp);  // odd_samples[0] = temp[0]

    temp = _mm_mul_ps(_mm_add_ps(odd_samples,old_odd_samples),coeffs); // all taps but the middle
    // Right shift old_odd_samples (no longer need old_odd_samples[0])
    // sets old_odd_samples[3] = 0 in preparation for ORing later with odd_samples[3]
    old_odd_samples = shiftright(old_odd_samples,1);

    // old_odd_samples[3] = odd_samples[3]
    // Shift right 3 words and then back to clear words 2-0
    old_odd_samples = _mm_add_ps(old_odd_samples,shiftleft(shiftright(odd_samples,3),3));

    // left shift odd samples
    odd_samples = shiftleft(odd_samples,1);

    // Sum up 4 terms
    temp = _mm_hadd_ps(temp,temp);
    temp = _mm_hadd_ps(temp,temp);    
    
    temp = _mm_add_ps(temp,shiftright(even_samples,3));       // temp += even_samples[3] (central tap, unity gain)

    // Left shift one word (no longer need even_samples[3])
    even_samples = shiftleft(even_samples,1);

    // Stash result:     *output++ = result;
    _mm_store_ss(output++,temp);
  }
  _mm_storeu_ps(state->even_samples,even_samples);
  _mm_storeu_ps(state->odd_samples,odd_samples);
  _mm_storeu_ps(state->old_odd_samples,old_odd_samples);
}

#else

// Portable version - written to help the compiler vectorize at least partly
void hb15_block(struct hb15_state *state,float *output,float *input,int cnt){
  float even_samples[4];
  float odd_samples[4];
  float old_odd_samples[4];
  float coeffs[4];

  memcpy(coeffs, state->coeffs, sizeof(coeffs));
  memcpy(even_samples, state->even_samples, sizeof(even_samples));
  memcpy(odd_samples, state->odd_samples, sizeof(odd_samples));
  memcpy(old_odd_samples, state->old_odd_samples, sizeof(old_odd_samples));

  while(cnt--){
    even_samples[0] = *input++;
    odd_samples[0] = *input++;

    float result = even_samples[3];
    for(int i=2; i >= 0; i--)
      even_samples[i+1] = even_samples[i];

    for(int i=0; i < 4; i++)
      result += (odd_samples[i] + old_odd_samples[i]) * coeffs[i];
    
    *output++ = result;

    for(int i=0; i < 3; i++)
      old_odd_samples[i] = old_odd_samples[i+1];
    
    old_odd_samples[3] = odd_samples[3];
    
    for(int i=2; i >= 0; i--)
      odd_samples[i+1] = odd_samples[i];
  }
  memcpy(state->even_samples, even_samples, sizeof(even_samples));
  memcpy(state->odd_samples, odd_samples, sizeof(odd_samples));
  memcpy(state->old_odd_samples, old_odd_samples, sizeof(old_odd_samples));
}
#endif
