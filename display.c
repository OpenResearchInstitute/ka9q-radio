// $Id: display.c,v 1.13 2017/05/29 18:35:03 karn Exp karn $
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

#include "radio.h"
#include "audio.h"
#include "dsp.h"

void *display(void *arg){
  WINDOW *fw,*prompt;
  WINDOW *sig;
  WINDOW *sdr;
  WINDOW *net;
  int c;
  struct demod *demod = &Demod;
  int tunestep = 0;

  initscr();
  keypad(stdscr,TRUE);
  timeout(100);
  cbreak();
  noecho();
  fw = newwin(5,30,0,0);
  sig = newwin(7,20,5,0);
  sdr = newwin(7,30,5,25);
  net = newwin(6,36,14,0);
  

  for(;;){
    wmove(fw,0,0);
    wprintw(fw,"Frequency   %'17.2f\n",get_first_LO(demod) - get_second_LO(demod,0) + Modes[demod->mode].dial);
    wprintw(fw,"First LO    %'17.2f\n",get_first_LO(demod));
    wprintw(fw,"First IF    %'17.2f\n",-get_second_LO(demod,0));
    wprintw(fw,"Dial offset %'17.2f\n",Modes[demod->mode].dial);
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
    mvwchgat(fw,0,x,1,A_STANDOUT,0,NULL);
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
    wprintw(sdr,"I offset %7.1f\n",demod->DC_i);
    wprintw(sdr,"Q offset %7.1f\n",demod->DC_q);
    wprintw(sdr,"I/Q imbal%7.1f dB\n",power2dB(demod->power_i/demod->power_q));
    wprintw(sdr,"I/Q phi  %10.5f\n",demod->sinphi);
    wprintw(sdr,"LNA      %7u\n",demod->lna_gain);
    wprintw(sdr,"Mix gain %7u\n",demod->mixer_gain);
    wprintw(sdr,"IF gain  %7u\n",demod->if_gain);
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
    c = getch();
    switch(c){
    case ERR:   // no key; timed out. Do nothing.
      continue;
    case 'q':   // Exit radio program
      goto done;
    case KEY_LEFT:
      if(tunestep < 8)
	tunestep++;
      break;
    case KEY_RIGHT:
      if(tunestep > -2)
	tunestep--;
      break;
    case KEY_UP:
      set_freq(demod,get_freq(demod) + pow(10.,tunestep),0);
      break;
    case KEY_DOWN:
      set_freq(demod,get_freq(demod) - pow(10.,tunestep),0);
      break;
    case '\f':  // Screen repaint
      clearok(curscr,TRUE);
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
