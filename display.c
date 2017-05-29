// $Id: display.c,v 1.11 2017/05/29 10:29:18 karn Exp karn $
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

  initscr();
  keypad(stdscr,TRUE);
  timeout(100);
  cbreak();
  noecho();
  fw = newwin(5,30,0,0);
  sig = newwin(6,20,5,0);
  sdr = newwin(7,30,5,25);
  net = newwin(6,35,12,0);
  

  for(;;){
    wmove(fw,0,0);
    wprintw(fw,"Frequency   %'17.2f\n",get_first_LO() - get_second_LO(0) + Modes[Demod.mode].dial);
    wprintw(fw,"First LO    %'17.2f\n",get_first_LO());
    wprintw(fw,"First IF    %'17.2f\n",-get_second_LO(0));
    wprintw(fw,"Dial offset %'17.2f\n",Modes[Demod.mode].dial);
    wrefresh(fw);

    wmove(sig,0,0);
    wprintw(sig,"Mode         %3s\n",Modes[Demod.mode].name);
    wprintw(sig,"IF1     %7.1f dB\n",power2dB(Demod.power_i + Demod.power_q));
    wprintw(sig,"IF2     %7.1f dB\n",voltage2dB(Demod.amplitude));
    wprintw(sig,"AF Gain %7.1f dB\n",voltage2dB(Demod.gain));
    wprintw(sig,"SNR     %7.1f dB\n",power2dB(Demod.snr));
    wrefresh(sig);
    
    wmove(sdr,0,0);
    wprintw(sdr,"I offset %7.1f\n",Demod.DC_i);
    wprintw(sdr,"Q offset %7.1f\n",Demod.DC_q);
    wprintw(sdr,"I/Q imbal%7.1f dB\n",power2dB(Demod.power_i/Demod.power_q));
    wprintw(sdr,"I/Q phi  %10.5f\n",Demod.sinphi);
    wprintw(sdr,"LNA      %7u\n",Demod.lna_gain);
    wprintw(sdr,"Mix gain %7u\n",Demod.mixer_gain);
    wprintw(sdr,"IF gain  %7u\n",Demod.if_gain);
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
    case '\f':  // Screen repaint
      clearok(curscr,TRUE);
      break;
    case 'n':   // Set noise reference to current amplitude; hit with no sig
      Demod.noise = Demod.amplitude;
      break;
    case 'm':
      prompt = newwin(3,50,20,0);
      box(prompt,0,0);
      mvwprintw(prompt,1,1,"Enter mode: ");
      wrefresh(prompt);
      echo();
      timeout(0);
      wgetnstr(prompt,str,sizeof(str));
      timeout(100);
      noecho();
      for(i=1;i <= Nmodes;i++){
	if(strcasecmp(str,Modes[i].name) == 0){
	  set_mode(Modes[i].mode);
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
      if(f > 0){
	set_second_LO(-48000,0);
	set_first_LO(f - 48000 - Modes[Demod.mode].dial,1);
      }
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
