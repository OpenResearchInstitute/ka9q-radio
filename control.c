// $Id$
// Send remote commands
#include <complex.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <locale.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#if linux
#include <linux/input.h>
#endif

#include "command.h"

#define DIAL "/dev/input/by-id/usb-Griffin_Technology__Inc._Griffin_PowerMate-event-if00"

double Tune_step = 1000;

enum dialmode {
  TUNE,
  MODE,
  STEP,
  LO,
  CAL,
} Dialmode;
#define NDIALMODES (CAL+1)

char *Dialmodes[] = { "Tune", "Mode", "Step", "LO", "CAL"};

int main(int argc,char *argv[]){
  int ctl;
  int r,i;
  char *locale;
  struct addrinfo *results,*rp,hints;
  char *destination = NULL;
  struct command command,ocommand;
  double freq;
#if linux
  int dial_fd;
  int button;
  struct timeval button_timeval;
  struct input_event event;
#endif

  locale = getenv("LANG");
  setlocale(LC_ALL,locale);

  // Scan arguments past options and process
  memset(&command,0,sizeof(command));
  command.cmd = SETSTATE;
  command.second_LO = -48000;
  command.first_LO = 147435000 + command.second_LO;
  command.second_LO_rate = 0;
  command.calibrate = 0;
  command.mode = FM; // Invalid

  while((r = getopt(argc,argv,"c:d:f:m:s:i:")) != EOF){
    switch(r){
    case 'i':
      command.second_LO = -atof(optarg);
      break;
    case 's':
      command.second_LO_rate = -atof(optarg);
      break;
    case 'm':
      for(i=1;i<=Nmodes;i++){
	if(strcasecmp(Modes[i].name,optarg) == 0){
	  command.mode = Modes[i].mode;
	  break;
	}
      }
      break;
    case 'f':
      freq = atof(optarg);
      break;
    case 'c':
      command.calibrate = atof(optarg) * 1e-6;
      break;
    case 'd':
      destination = optarg;
      break;
    }
  }
  command.first_LO = freq + command.second_LO;

  memset(&hints,0,sizeof(hints));
  hints.ai_flags |= (AI_V4MAPPED|AI_ADDRCONFIG);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = 0;
  
  if((r = getaddrinfo(destination,"4159",&hints,&results)) != 0){
    fprintf(stderr,"%s: %s\n",destination,gai_strerror(r));
    exit(1);
  }
  for(rp = results;rp != NULL; rp = rp->ai_next){
    if((ctl = socket(rp->ai_family,rp->ai_socktype, 0)) == -1)
      continue;
    if(connect(ctl,rp->ai_addr,rp->ai_addrlen) == 0)
      break;
    close(ctl);
  }
  if(rp == NULL){
    fprintf(stderr,"Connect to %s failed\n",destination);
    exit(1);
  }

  freeaddrinfo(results);
  results = NULL;

  write(ctl,&command,sizeof(command));

#if linux
  ocommand = command;
  // If a dial is attached, go into a loop handling it. Otherwise exit.
  if((dial_fd = open(DIAL,O_RDONLY)) < 0){
    fprintf(stderr,"No Griffin Powermate!\n");
    exit(1);
  }
  fprintf(stderr,"Griffin Powermate found\n");

  gettimeofday(&button_timeval,NULL);
  button = 0;
  while((r = read(dial_fd,&event,sizeof(event))) == sizeof(event)){
    double adjust;

    if(r == 0)
      continue;

    switch(event.type){
    case EV_SYN:
      continue; // Completely ignore; do not fall through switch and send status query
    case EV_REL: // Relative dial motion
      switch(event.code){
      case REL_DIAL:
	if(button) // Ignore turns when button pushed (easy to do by accident)
	  break;
	switch(Dialmode){
	case CAL:  // adjust frequency calibration
	  command.calibrate += event.value / 1e8;
	  break;
	case LO:
	  // shift first LO while keeping constant tuning frequency
	  // Limit range, allowing 1 Hz for first LO error adjustment
	  if(fabs(command.second_LO - event.value * Tune_step) < 95999){
	    freq = command.first_LO - command.second_LO;
	    command.second_LO -= event.value * Tune_step;
	    command.first_LO = freq + command.second_LO;
	  }
	  break;
	case TUNE:
	  adjust = 0;
	  command.second_LO -= event.value * Tune_step;
	  if(-command.second_LO + Modes[command.mode].high >= 96000){
	    adjust = 192000 - (Modes[command.mode].high - Modes[command.mode].low);
	  } else if(-command.second_LO + Modes[command.mode].low <= -96000){
	    adjust = -192000 + (Modes[command.mode].high - Modes[command.mode].low);
	  }
	  command.second_LO += adjust;
	  command.first_LO += adjust;
	  break;
	case MODE:
	  command.mode += event.value;
	  if(command.mode <= 0)
	    command.mode = Nmodes;
	  else if(command.mode > Nmodes)
	    command.mode = 0;
	  break;
	case STEP:
	  if((event.value < 0 && Tune_step > 0.01) || (event.value > 0 && Tune_step < 10e6))
	    Tune_step *= pow(10,event.value);
	  fprintf(stderr,"Tune Step = %'.2lf\n",Tune_step);
	  break;
	} // switch (Dialmode)
	break;
      } // switch(event.code)
      break;
    case EV_KEY: // Dial button press or release
      switch(event.code){
      case BTN_MISC:
	// My Powermate has developed an annoying habit of generating spurious button presses
	// They're all short, so act only if it's long enough
	if(event.value){
	  // Button pushed
	  button_timeval = event.time; // remember when it was pushed
	  button = 1;
	} else {
	  int downtime;
	  button = 0;	  // Button released
	  downtime = 1000000*(event.time.tv_sec - button_timeval.tv_sec) + (event.time.tv_usec - button_timeval.tv_usec);
	  if(downtime > 50000){ // 50 millisec
	    Dialmode = (Dialmode + 1) % NDIALMODES;
	    fprintf(stderr,"Dialmode %s\n",Dialmodes[Dialmode]);
	  }
	}
	break;
      } // switch(event.code)
      break;
    } // switch(event.type)
    if(memcmp(&command,&ocommand,sizeof(command)) != 0){
#if 1
      fprintf(stderr,"mode %d 1stLO %'.02lf 2ndLO %'.02lf 2ndLOrate %'.02lf cal %'lg\n",
	      (int)command.mode,command.first_LO,command.second_LO,command.second_LO_rate,command.calibrate);
#endif

      write(ctl,&command,sizeof(command));    
      ocommand = command;
    }
  } // read(dial)
  fprintf(stderr,"Dial read error %s\n",strerror(errno));
  close(dial_fd);
  dial_fd = -1;
#endif
  close(ctl);
  ctl = -1;
  exit(0);
}
