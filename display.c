// $Id: display.c,v 1.14 2017/05/31 22:26:52 karn Exp karn $
// Thread to display internal state of 'radio' command on command line 
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
//#include <linux/input.h> // definitions conflict with ncurses.h !!

#include "radio.h"
#include "audio.h"
#include "dsp.h"

#define DIAL "/dev/input/by-id/usb-Griffin_Technology__Inc._Griffin_PowerMate-event-if00"

void *display(void *arg){
  WINDOW *fw,*prompt;
  WINDOW *sig;
  WINDOW *sdr;
  WINDOW *net;
  int c;
  struct demod *demod = &Demod;
  int tunestep = 0;
  int tuneitem = 0;
  int dial_fd;

  initscr();
  keypad(stdscr,TRUE);
  timeout(100);
  cbreak();
  noecho();
  fw = newwin(5,40,0,0);
  sig = newwin(7,20,6,0);
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

    wmove(sig,0,0);
    wprintw(sig,"Mode         %3s\n",Modes[demod->mode].name);
    wprintw(sig,"IF1     %7.1f dB\n",power2dB(demod->power_i + demod->power_q));
    wprintw(sig,"IF2     %7.1f dB\n",voltage2dB(demod->amplitude));
    wprintw(sig,"AF Gain %7.1f dB\n",voltage2dB(demod->gain));
    wprintw(sig,"SNR     %7.1f dB\n",power2dB(demod->snr));
    wprintw(sig,"offset  %7.1f Hz\n",demod->samprate/demod->decimate * demod->foffset/(2*M_PI));
    wprintw(sig,"deviat  %7.1f Hz\n",demod->samprate/demod->decimate * demod->pdeviation/(2*M_PI));
    wrefresh(sig);
    
    wmove(sdr,0,0);
    wprintw(sdr,"I offset %10.1f\n",demod->DC_i);
    wprintw(sdr,"Q offset %10.1f\n",demod->DC_q);
    wprintw(sdr,"I/Q imbal%10.1f dB\n",power2dB(demod->power_i/demod->power_q));
    wprintw(sdr,"I/Q phi  %10.5f rad\n",demod->sinphi);
    wprintw(sdr,"LNA      %10u\n",demod->lna_gain);
    wprintw(sdr,"Mix gain %10u\n",demod->mixer_gain);
    wprintw(sdr,"IF gain  %10u dB\n",demod->if_gain);
    wrefresh(sdr);
    
    extern int Olds,Skips;

    wmove(net,0,0);
    wprintw(net,"Olds %d\n",Olds);
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
    char str[80];
    int i;

    
#if 1
    struct input_event {
      struct timeval time;
      uint16_t type;
      uint16_t code;
      int32_t value;
    };

    struct input_event event;
    if(read(dial_fd,&event,sizeof(event)) == sizeof(event)){
      // Got something from the powermate knob
      switch(event.type){
      case 0:   //case EV_SYN:
	continue; // Ignore
      case 2: // case EV_REL:
	switch(event.code){
	case 7: // REL_DIAL:
	  if(event.value > 0)
	    c = KEY_UP;
	  else if(event.value < 0)
	    c = KEY_DOWN;
	  break;
	}
	break;
      case 1: // EV_KEY:
	switch(event.code){
	case 0x100: // BTN_MISC:
	  if(event.value){
	    set_freq(demod,get_freq(demod),1);
	  }
	  break;
	default:
	  break;
	}
      default:
	break;
      }
    } else
#endif
      c = getch();

    switch(c){
    case ERR:   // no key; timed out. Do nothing.
      continue;
    case 'q':   // Exit radio program
      goto done;
    case '\t':
      tuneitem = (tuneitem + 1) % 5;
      break;
    case KEY_BTAB:
      tuneitem = (5 + tuneitem - 1) % 5;
      break;
    case KEY_HOME:
      tuneitem = 0;
      tunestep = 0;
      break;
    case KEY_BACKSPACE:
    case KEY_LEFT:
      if(tunestep < 8)
	tunestep++;
      break;
    case KEY_RIGHT:
      if(tunestep > -2)
	tunestep--;
      break;
    case KEY_UP:
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
    case KEY_DOWN:
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
    case 'c':
      prompt = newwin(3,50,20,0);
      box(prompt,0,0);
      mvwprintw(prompt,1,1,"Enter calibrate offset in ppm: ");
      wrefresh(prompt);
      echo();
      timeout(0);
      wgetnstr(prompt,str,sizeof(str));
      timeout(100);
      noecho();
      f = atof(str);
      set_cal(demod,f * 1e-6);

      werase(prompt);
      wrefresh(prompt);
      delwin(prompt);
      break;
    case 'n':   // Set noise reference to current amplitude; hit with no sig
      demod->noise = demod->amplitude;
      break;
    case 'm':
      prompt = newwin(3,80,20,0);
      box(prompt,0,0);
      mvwprintw(prompt,1,1,"Enter mode [");
      for(i=1;i <= Nmodes;i++)
	wprintw(prompt," %s",Modes[i].name);
      wprintw(prompt," ]: ");
      wrefresh(prompt);
      echo();
      timeout(0);
      wgetnstr(prompt,str,sizeof(str));
      timeout(100);
      noecho();
      for(i=1;i <= Nmodes;i++){
	if(strcasecmp(str,Modes[i].name) == 0){
	  set_mode(demod,Modes[i].mode);
	  break;
	}
      }
      werase(prompt);
      wrefresh(prompt);
      delwin(prompt);
      break;
    case 'f':   // Tune to new frequency
      prompt = newwin(3,50,20,0);
      box(prompt,0,0);
      mvwprintw(prompt,1,1,"Enter frequency in Hz: ");
      wrefresh(prompt);
      echo();
      timeout(0);
      wgetnstr(prompt,str,sizeof(str));
      timeout(100);
      noecho();
      f = atof(str);
      if(f > 0)
	set_freq(demod,f,1);

      werase(prompt);
      wrefresh(prompt);
      delwin(prompt);
      break;
    default:
      //      fprintf(stderr,"char %d 0x%x",c,c);
      break;
    }
  }
 done:;
  werase(fw);
  wrefresh(fw);

  werase(sig);
  wrefresh(sig);

  werase(sdr);
  wrefresh(sdr);

  werase(net);
  wrefresh(net);
  
  echo();
  nocbreak();

  endwin();

  exit(0);
  pthread_exit(0);
}
