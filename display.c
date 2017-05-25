// $Id: display.c,v 1.9 2017/05/25 05:11:25 karn Exp karn $
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
  int c;

  initscr();
  keypad(stdscr,TRUE);
  timeout(100);
  cbreak();
  noecho();
  fw = newwin(5,30,0,0);
  sig = newwin(6,20,6,0);
  sdr = newwin(7,30,6,25);

  for(;;){
    mvwprintw(fw,0,0,"Frequency   %'17.2f",get_first_LO() - get_second_LO(0) + Modes[Demod.mode].dial);
    mvwprintw(fw,1,0,"First LO    %'17.2f",get_first_LO());
    mvwprintw(fw,2,0,"First IF    %'17.2f",-get_second_LO(0));
    mvwprintw(fw,3,0,"Dial offset %'17.2f",Modes[Demod.mode].dial);
    wrefresh(fw);

    mvwprintw(sig,0,0,"Mode         %3s",Modes[Demod.mode].name);
    mvwprintw(sig,1,0,"IF1     %7.1f dB",power2dB(Demod.power_i + Demod.power_q));
    mvwprintw(sig,2,0,"IF2     %7.1f dB",voltage2dB(Demod.amplitude));
    mvwprintw(sig,3,0,"AF Gain %7.1f dB",voltage2dB(Demod.gain));
    mvwprintw(sig,4,0,"SNR     %7.1f dB",power2dB(Demod.snr));
    wrefresh(sig);
    
    mvwprintw(sdr,0,0,"I offset %7.1f",Demod.DC_i);
    mvwprintw(sdr,1,0,"Q offset %7.1f",Demod.DC_q);
    mvwprintw(sdr,2,0,"I/Q imbal%7.1f dB",power2dB(Demod.power_i/Demod.power_q));
    mvwprintw(sdr,3,0,"I/Q phi  %10.5f",Demod.sinphi);
    mvwprintw(sdr,4,0,"LNA      %7u",Demod.lna_gain);
    mvwprintw(sdr,5,0,"Mix gain %7u",Demod.mixer_gain);
    mvwprintw(sdr,6,0,"IF gain  %7u",Demod.if_gain);
    wrefresh(sdr);
    

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
	set_first_LO(f - 48000,1);
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

  echo();
  nocbreak();

  endwin();

  exit(0);
  pthread_exit(0);
}
