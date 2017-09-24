// $Id$
// Real-time doppler steering
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

void *doppler(void *arg){
  pthread_setname("doppler");
  assert(arg != NULL);
  struct demod * const demod = arg;

  FILE *input;
  char line[1024];
  double t,az,azrate,el,elrate,range,rangerate,rangeraterate;
  double rt;

  pthread_mutex_lock(&demod->doppler_mutex);
  demod->doppler_phasor = 1;
  pthread_mutex_unlock(&demod->doppler_mutex);

  input = fopen("/tmp/tracking","r");
  while(1){
    fgets(line,sizeof(line),input);
    int n = sscanf(line,"%lf %lf %lf %lf %lf %lf %lf %lf",
	   &t,&az,&azrate,&el,&elrate,&range,&rangerate,&rangeraterate);
    if(n != 8)
      continue;
    
    struct timeval tv;
    gettimeofday(&tv,NULL);
    rt = tv.tv_sec + tv.tv_usec * 1e-6;
    if(t < rt)
      continue;
    if(t > rt){
      usleep(1000000*(t - rt)); // Wait until right time
    }
    // Compute doppler and doppler rate
    double const c = 299792458;
    pthread_mutex_lock(&demod->doppler_mutex);
    demod->doppler = demod->frequency * -rangerate/c;
    demod->doppler_rate = demod->frequency * -rangeraterate/c;
    demod->doppler_phasor_step = csincos(-2*M_PI*demod->doppler / demod->samprate);
    demod->doppler_phasor_step_step = csincos(-2*M_PI*demod->doppler_rate / (demod->samprate*demod->samprate));
    pthread_mutex_unlock(&demod->doppler_mutex);
  }
  pthread_exit(NULL);
}
