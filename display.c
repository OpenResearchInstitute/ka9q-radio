// $Id: display.c,v 1.19 2017/06/02 12:06:06 karn Exp karn $
// Thread to display internal state of 'radio' and accept single-letter commands
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#undef I
#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
// We need some of these definitions for the Griffin dial, but some conflict
// with definitions in ncurses.h !!arrrrggggh!!
// So we'll declare ourselves the small parts we need later
//#include <linux/input.h>

#include "radio.h"
#include "audio.h"
#include "dsp.h"

#define DIAL "/dev/input/by-id/usb-Griffin_Technology__Inc._Griffin_PowerMate-event-if00"

// Pop up a dialog box, issue a prompt and get a response
void getentry(char *prompt,char *response,int len){
  WINDOW *pwin;

  pwin = newwin(3,70,19,0);
  box(pwin,0,0);
  mvwprintw(pwin,1,1,prompt);
  wrefresh(pwin);
  echo();
  timeout(0);
  wgetnstr(pwin,response,len);
  timeout(100);
  noecho();
  werase(pwin);
  wrefresh(pwin);
  delwin(pwin);
}


void *display_cleanup(void *arg){
  echo();
  nocbreak();
  endwin();
  return NULL;
}

WINDOW *deb;

void *display(void *arg){
  int c;
  struct demod *demod = &Demod;
  int tunestep = 0;
  int tuneitem = 0;
  int dial_fd;
  WINDOW *fw;
  WINDOW *sig;
  WINDOW *sdr;
  WINDOW *net;

  pthread_cleanup_push(display_cleanup,demod);

  initscr();
  keypad(stdscr,TRUE);
  timeout(100); // This sets the update interval when nothing is typed
  cbreak();
  noecho();
  deb = newwin(5,70,20,0);
  scrollok(deb,1);
  fw = newwin(5,40,0,0);
  sig = newwin(7,25,6,0);
  sdr = newwin(7,30,6,25);
  net = newwin(6,36,14,0);
  
  dial_fd = open(DIAL,O_RDONLY|O_NDELAY);

  for(;;){
    wmove(fw,0,0);
    wprintw(fw,"Frequency   %'17.2f Hz\n",get_first_LO(demod) - get_second_LO(demod,0) + demod->dial_offset);
    wprintw(fw,"First LO    %'17.2f Hz\n",get_first_LO(demod));
    wprintw(fw,"First IF    %'17.2f Hz\n",-get_second_LO(demod,0));
    wprintw(fw,"Dial offset %'17.2f Hz\n",demod->dial_offset);
    wprintw(fw,"Calibrate   %'17.2f ppm\n",get_cal(demod)*1e6);
    // Tuning step highlight
    // A little messy because of the commas in the frequencies
    // They come from the ' option in the printf formats
    int x;
    if(tunestep >= -2 && tunestep <= -1){
      x = 25 - tunestep + 1;
    } else if(tunestep >= 0 && tunestep <= 2){
      x = 25 - tunestep;
    } else if(tunestep >= 3 && tunestep <= 5){
      x = 25 - tunestep - 1;
    } else if(tunestep >= 6 && tunestep <= 8){
      x = 25 - tunestep - 2;
    }
    mvwchgat(fw,tuneitem,x,1,A_STANDOUT,0,NULL);
    wrefresh(fw);

    wclrtobot(sig);     // clear previous stuff in case we stop showing the last lines
    wmove(sig,0,0);

    wprintw(sig,"Mode         %3s\n",Modes[demod->mode].name);
    wprintw(sig,"IF1     %7.1f dBFS\n",power2dB(demod->power_i + demod->power_q));
    wprintw(sig,"IF2     %7.1f dBFS\n",voltage2dB(demod->amplitude));
    wprintw(sig,"AF Gain %7.1f dB\n",voltage2dB(demod->gain));
    if(!isnan(demod->mode) && demod->snr != 0)
      wprintw(sig,"SNR     %7.1f dB\n",power2dB(demod->snr));
    if(!isnan(demod->foffset))
      wprintw(sig,"offset  %7.1f Hz\n",demod->samprate/demod->decimate * demod->foffset/(2*M_PI));
    if(!isnan(demod->pdeviation))
      wprintw(sig,"deviat  %7.1f Hz\n",demod->samprate/demod->decimate * demod->pdeviation/(2*M_PI));
    wrefresh(sig);
    
    wmove(sdr,0,0);
    wprintw(sdr,"I offset %10.1f\n",demod->DC_i);
    wprintw(sdr,"Q offset %10.1f\n",demod->DC_q);
    wprintw(sdr,"I/Q imbal%10.1f dB\n",power2dB(demod->power_i/demod->power_q));
    double sinphi = demod->dotprod / (demod->power_i + demod->power_q);
    wprintw(sdr,"I/Q phi  %10.5f rad\n",sinphi);
    wprintw(sdr,"LNA      %10u\n",demod->lna_gain);
    wprintw(sdr,"Mix gain %10u\n",demod->mixer_gain);
    wprintw(sdr,"IF gain  %10u dB\n",demod->if_gain);
    wrefresh(sdr);
    
    extern int Delayed,Skips;

    wmove(net,0,0);
    wprintw(net,"Delayed %d\n",Delayed);
    wprintw(net,"Skips %d\n",Skips);
    wprintw(net,"audio underrun %d\n",Audio.underrun);
    snd_pcm_sframes_t delayp;
    if(Audio.handle != NULL){
      snd_pcm_delay(Audio.handle,&delayp);
      wprintw(net,"audio delay %d (%.1f ms)\n",(int)delayp,1000.*(float)delayp/Audio.samprate);
      wprintw(net,"audio overflow %d\n",Audio.overflow);
    }
    wrefresh(net);

    double f;
    char str[160];
    int i;

    
    // Redefine stuff we need from linux/input.h
    // We can't include it because it conflicts with ncurses.h!
#define EV_SYN 0
#define EV_KEY 1
#define EV_REL 2
#define REL_DIAL 7    
#define BTN_MISC 0x100
    struct input_event {
      struct timeval time;
      uint16_t type;
      uint16_t code;
      int32_t value;
    };
    // End of redefined input stuff

    struct input_event event;
    if(read(dial_fd,&event,sizeof(event)) == sizeof(event)){
      // Got something from the powermate knob
      wprintw(deb,"type %d code %d value %d\n",event.type,event.code,event.value);
      wrefresh(deb);
      if(event.type == EV_SYN){
	// Ignore
	continue;
      } if(event.type == EV_REL){
	if(event.code == REL_DIAL){
	  // Dial has been turned. Simulate up/down arrow tuning commands
	  if(event.value > 0)
	    c = KEY_UP;
	  else if(event.value < 0)
	    c = KEY_DOWN;
	}
      } else if(event.type == EV_KEY){
	if(event.code == BTN_MISC)
	  if(event.value)
	    set_freq(demod,get_freq(demod),1);
      }
    } else
      c = getch(); // read with timeout from keyboard

    switch(c){
    case ERR:   // no key; timed out. Do nothing.
      continue;
    case 'q':   // Exit radio program
      goto done;
    case '\t':  // tab: cycle through tuning fields
      tuneitem = (tuneitem + 1) % 5;
      break;
    case KEY_BTAB: // Backtab, i.e., shifted tab: cycle backwards through tuning fields
      tuneitem = (5 + tuneitem - 1) % 5;
      break;
    case KEY_HOME: // Go back to starting spot
      tuneitem = 0;
      tunestep = 0;
      break;
    case KEY_BACKSPACE: // Cursor left: increase tuning step 10x
    case KEY_LEFT:
      if(tunestep < 8)
	tunestep++;
      break;
    case KEY_RIGHT:     // Cursor right: decrease tuning step /10
      if(tunestep > -2)
	tunestep--;
      break;
    case KEY_UP:        // Increase whatever we're tuning
      switch(tuneitem){
      case 0:
	set_freq(demod,get_freq(demod) + pow(10.,tunestep),0);
	break;
      case 1:
	set_first_LO(demod,get_first_LO(demod) + pow(10.,tunestep),0);
	break;
      case 2:
	set_second_LO(demod,get_second_LO(demod,0) - pow(10.,tunestep),0);
	break;
      case 3:
	demod->dial_offset += pow(10.,tunestep);
	break;
      case 4:
	set_cal(demod,get_cal(demod) + pow(10.,tunestep) * 1e-6);
	break;
      }
      break;
    case KEY_DOWN:      // Decrease whatever we're tuning
      switch(tuneitem){
      case 0:
	set_freq(demod,get_freq(demod) - pow(10.,tunestep),0);
	break;
      case 1:
	set_first_LO(demod,get_first_LO(demod) - pow(10.,tunestep),0);
	break;
      case 2:
	set_second_LO(demod,get_second_LO(demod,0) + pow(10.,tunestep),0);
	break;
      case 3:
	demod->dial_offset -= pow(10.,tunestep);
	break;
      case 4:
	set_cal(demod,get_cal(demod) - pow(10.,tunestep) * 1e-6);
	break;
      }
      break;
    case '\f':  // Screen repaint
      clearok(curscr,TRUE);
      break;
    case 'c':   // TCXO calibration offset
      getentry("Enter calibrate offset in ppm: ",str,sizeof(str));
      if(strlen(str) > 0){
	f = atof(str);
	set_cal(demod,f * 1e-6);
      }
      break;
    case 'n':   // Set noise reference to current amplitude; hit with no sig
      demod->noise = demod->amplitude;
      break;
    case 'm':   // Select demod mode from list
      strncpy(str,"Enter mode [ ",sizeof(str));
      for(i=1;i <= Nmodes;i++){
	strncat(str,Modes[i].name,sizeof(str) - strlen(str));
	strncat(str," ",sizeof(str) - strlen(str));
      }
      strncat(str,"]: ",sizeof(str) - strlen(str));
      getentry(str,str,sizeof(str));
      if(strlen(str) > 0){
	for(i=1;i <= Nmodes;i++){
	  if(strcasecmp(str,Modes[i].name) == 0){
	    set_mode(demod,Modes[i].mode);
	    break;
	  }
	} 
      }
      break;
    case 'f':   // Tune to new frequency
      getentry("Enter frequency in Hz: ",str,sizeof(str));
      if(strlen(str) > 0){
	f = atof(str);
	if(f > 0)
	set_freq(demod,f,1);
      }
      break;
    default:
      //      fprintf(stderr,"char %d 0x%x",c,c);
      break;
    }
  }
 done:;
  pthread_cleanup_pop(1);
  exit(0);
  pthread_exit(NULL);
}
