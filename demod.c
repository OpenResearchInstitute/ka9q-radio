// $Id$
// Common demod thread for all modes
// Takes commands from UDP packets on a socket
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


int Front_end_ctl;
extern char *Tuner_host;

void *demod_loop(void *arg){
  int ctl,r;
  char pktbuf[1500];
  struct sockaddr_in6 address;
  struct addrinfo *results,*rp,hints;
  char *destination = NULL;
  struct command *command = arg;
  
  // Set up incoming control socket
  address.sin6_family = AF_INET6;
  address.sin6_port = htons(4159);
  address.sin6_flowinfo = 0;
  address.sin6_addr = in6addr_any;
  address.sin6_scope_id = 0;

  if((ctl = socket(PF_INET6,SOCK_DGRAM, 0)) == -1){
    fprintf(stderr,"demod_loop: can't open control socket\n");
    exit(1);
  }
  if(bind(ctl,&address,sizeof(address)) != 0){
    fprintf(stderr,"demod_loop: bind failed\n");
    exit(1);
  }
  fcntl(ctl,F_SETFL,O_NONBLOCK);

  // Set up outgoing socket to control front end
  memset(&hints,0,sizeof(hints));
  hints.ai_flags |= (AI_V4MAPPED|AI_ADDRCONFIG);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = 0;
  
  if((r = getaddrinfo(Tuner_host,"6767",&hints,&results)) != 0){
    fprintf(stderr,"%s: %s\n",destination,gai_strerror(r));
    exit(1);
  }
  for(rp = results;rp != NULL; rp = rp->ai_next){
    if((Front_end_ctl = socket(rp->ai_family,rp->ai_socktype, 0)) == -1)
      continue;
    if(connect(Front_end_ctl,rp->ai_addr,rp->ai_addrlen) == 0)
      break;
    close(Front_end_ctl);
  }
  if(rp == NULL){
    fprintf(stderr,"Connect to %s failed\n",destination);
    exit(1);
  }
  freeaddrinfo(results);
  results = NULL;
  assert(Front_end_ctl > 0);

  // Set up to send a message to ourselves
  address.sin6_addr = in6addr_loopback;

  // Send ourselves and the tuner the command
  write(Front_end_ctl,command,sizeof(*command)); // not really necessary
  sendto(ctl,command,sizeof(*command),0,&address,sizeof(address)); // to ourselves

  while(1){
    int rdlen,n;
    socklen_t addrlen;
    float energy;
    
    // See if we have any commands
    while(addrlen = sizeof(address), (rdlen = recvfrom(ctl,&pktbuf,sizeof(pktbuf),0,&address,&addrlen)) > 0)
      process_command(pktbuf,rdlen);

    if(Demod.filter == NULL || Demod.filter->input == NULL){
      // We're not initialized yet
      usleep(500000);
      continue;
    }
    if(read(0,Demod.filter->input,Demod.filter->blocksize_in * sizeof(complex float)) <= 0)
      break;
    energy = 0;
    for(n=0; n < Demod.filter->blocksize_in; n++){
      energy += crealf(Demod.filter->input[n]) * crealf(Demod.filter->input[n]) +
	cimagf(Demod.filter->input[n]) * cimagf(Demod.filter->input[n]);
    }
    Demod.energy = energy /= Demod.filter->blocksize_in;
    spindown(Demod.filter->input,Demod.filter->blocksize_in); // 2nd LO
    execute_filter(Demod.filter);
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
  close(Front_end_ctl);
  close(ctl);
  return NULL;
}

int process_command(char *cmdbuf,int len){
  if(len >= sizeof(struct command)){
    struct command command;

    memcpy(&command,cmdbuf,sizeof(command));
    switch(command.cmd){
    case SENDSTAT:
      break; // do this later
    case SETSTATE: // first LO freq, second LO freq, second_LO freq rate, mode
#if 0
      fprintf(stderr,"setstate(%d,%.2lf,%.2lf,%.2lf,%.2lf)\n",
	      command.mode,
	      command.first_LO,
	      command.second_LO,command.second_LO_rate,command.calibrate);
#endif
      // Ignore out-of-range values
      if(command.first_LO > 0 && command.first_LO < 2e9)
	set_first_LO(command.first_LO,0);
      if(command.second_LO >= -Demod.samprate/2 && command.second_LO <= +Demod.samprate/2)
	set_second_LO(command.second_LO,0);
      if(fabs(command.second_LO_rate) < 1e9)
	set_second_LO_rate(command.second_LO_rate,0);
      if(command.mode > 0 && command.mode <= Nmodes)
	set_mode(command.mode);
      if(fabs(command.calibrate) < 1)
	set_cal(command.calibrate);
      break;
    default:
      break; // Ignore
    } // switch
  }
  return 0;
}

int process_ssb(){
  // Automatic gain control
  Demod.amplitude = amplitude(Demod.filter->output.r,Demod.filter->blocksize_out);
  ssb_agc();
  
  put_mono_audio(Demod.filter->output.r,Demod.filter->blocksize_out,Demod.gain);
  return 0;
}

// Envelope detection: take magnitude of complex samples, ignoring phase
// Also find average carrier level for AGC and DC removal
const double Headroom = 0.3;
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
  float gain = Demod.gain / M_PI;

  // If squelch is closed, just let the output drain
  Demod.snr = squelch_fm(Demod.filter->output.c,Demod.filter->blocksize_out) - 1;

  if(Demod.snr > 2){
    do_fm(audio,Demod.filter->output.c,Demod.filter->blocksize_out,&Demod.fmstate);
    put_mono_audio(audio,Demod.filter->blocksize_out,gain);
  }
  return 0;
}
int process_wfm(){
  float audio[Demod.filter->blocksize_in];
  float gain = Demod.gain / M_PI;

  // If squelch is closed, just let the output drain
  // We don't use a pre-demod filter, so we work with the *INPUT* buffer only
  if((Demod.snr = squelch_fm(Demod.filter->input,Demod.filter->blocksize_in) - 1) < 2)
    return 0;

  do_fm(audio,Demod.filter->input,Demod.filter->blocksize_in,&Demod.fmstate);
  put_mono_audio(audio,Demod.filter->blocksize_in,gain);
  return 0;
}
