// $Id: doppler.c,v 1.2 2017/09/26 09:34:24 karn Exp karn $
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

  set_doppler(demod,0,0);

  while(1){
    input = fopen("/tmp/tracking","r");
    if(input == NULL){
      usleep(100000); // Don't spin tight
      continue;
    }
      
    while(fgets(line,sizeof(line),input) != NULL){
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
      double const f = get_freq(demod);
      set_doppler(demod,f * -rangerate/c,f * -rangeraterate/c);
    }
    fclose(input); // and try again

    set_doppler(demod,0,0);
  }
  pthread_exit(NULL);
}
