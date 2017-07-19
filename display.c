// $Id: display.c,v 1.48 2017/07/18 00:40:08 karn Exp karn $
// Thread to display internal state of 'radio' and accept single-letter commands
// Copyright 2017 Phil Karn, KA9Q - may be used under the Gnu Public License v2.
#define _GNU_SOURCE 1
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#undef I
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ncurses.h>
#include <ctype.h>
// We need some of these definitions for the Griffin dial, but some conflict
// with definitions in ncurses.h !!arrrrggggh!!
// So we'll declare ourselves the small parts we need later
//#include <linux/input.h>

#include "radio.h"
#include "audio.h"
#include "dsp.h"
#include "filter.h"
#include "bandplan.h"

#define DIAL "/dev/input/by-id/usb-Griffin_Technology__Inc._Griffin_PowerMate-event-if00"

float Spare; // General purpose knob for experiments

int Update_interval = 100;
int Tunestep;


// Pop up a temporary window with the contents of a file,
// then wait for a kb character to clear it
#define MAXCOLS 256
void popup(const char *filename){

  FILE *fp;
  if((fp = fopen(filename,"r")) == NULL)
    return;
  // Determine size of box
  int rows=0, cols=0;
  char line[MAXCOLS];
  while(fgets(line,sizeof(line),fp) != NULL){
    chomp(line);
    rows++;
    if(strlen(line) > cols)
      cols = strlen(line);
  }
  rewind(fp);
  
  // Allow room for box
  WINDOW * const pop = newwin(rows+2,cols+2,0,0);
  box(pop,0,0);
  int row = 0;
  while(fgets(line,sizeof(line),fp) != NULL){
    chomp(line);
    mvwprintw(pop,++row,1,line);
  }
  fclose(fp);
  wnoutrefresh(pop);
  doupdate();
  timeout(-1); // blocking read - wait indefinitely
  (void)getch(); // Read and discard one character
  timeout(Update_interval);
  werase(pop);
  wrefresh(pop);
  delwin(pop);
}


// Pop up a dialog box, issue a prompt and get a response
void getentry(char const *prompt,char *response,int len){
  WINDOW *pwin = newwin(3,70,20,0);
  box(pwin,0,0);
  mvwprintw(pwin,1,1,prompt);
  wrefresh(pwin);
  echo();
  timeout(-1);
  // Manpage for wgetnstr doesn't say whether a terminating
  // null is stashed. Hard to believe it isn't, but this is to be sure
  memset(response,0,len);
  wgetnstr(pwin,response,len);
  chomp(response);
  timeout(Update_interval);
  noecho();
  werase(pwin);
  wrefresh(pwin);
  delwin(pwin);
}


void display_cleanup(void *arg){
  echo();
  nocbreak();
  endwin();
}

WINDOW *deb;

// Adjust the selected item up or down one step
static void adjust_item(struct demod *demod,const int tuneitem,const double tunestep){
  switch(tuneitem){
  case 0: // Dial frequency
    if(!demod->frequency_lock) // Ignore if locked
      set_freq(demod,get_freq(demod) + tunestep,0);
    break;
  case 1: // First LO
    if(fabs(tunestep) < 1)
      break; // First LO can't make steps < 1  Hz
    
    if(demod->frequency_lock){
      // See how much we'll have to retune LO2 to stay on frequency
      double new_lo2 = get_second_LO(demod,0) + tunestep * (1 + demod->calibrate);
      if(LO2_in_range(demod,new_lo2,0)){
	// This waits for it to actually change, minimizing the glitch that would
	// otherwise happen if we changed LO2 first
	set_first_LO(demod, get_first_LO(demod) + tunestep * (1 + demod->calibrate),0);
	// Change LO2 by actual amount of LO1 change
	set_second_LO(demod,new_lo2);
      } // else ignore; LO2 would be out-of-range
    } else
      set_first_LO(demod, get_first_LO(demod) + tunestep * (1 + demod->calibrate),0);

    break;
  case 2: // IF
    ; // needed because next line is declaration
    double new_lo2 = get_second_LO(demod,0);

    if(demod->frequency_lock){
      if(fabs(tunestep) < 1)
	break;	  // First LO can't maintain locked dial frequency with steps < 1  Hz
      
      new_lo2 -= tunestep * (1 + demod->calibrate); // Accomodate actual change in LO1
      if(!LO2_in_range(demod,new_lo2,0))
	break; // Ignore command to tune IF out of range
      set_first_LO(demod, get_first_LO(demod) - tunestep * (1 + demod->calibrate),0);
    } else {
      new_lo2 -= tunestep;
    }
    if(LO2_in_range(demod,new_lo2,0))
      set_second_LO(demod,new_lo2); // Ignore if out of range
    break;
  case 3: // Dial offset
    demod->dial_offset += tunestep;
    break;
  case 4: // Calibrate offset
    set_cal(demod,get_cal(demod) + tunestep * 1e-6); // ppm
    break;
  case 5: // Filter low edge
    demod->low += tunestep;
    set_filter(demod,demod->low,demod->high);
    break;
  case 6: // Filter high edge
    demod->high += tunestep;
    set_filter(demod,demod->low,demod->high);
    break;
  case 7: // Kaiser window beta parameter for filter
    if(Kaiser_beta + tunestep >= 0.0){
      Kaiser_beta += tunestep;
      set_filter(demod,demod->low,demod->high); // Recreate filters
    }
    break;
  case 8: // Spare for experimentation
    Spare += tunestep;
    break;
  }
}

// Thread to display receiver state, updated at 10Hz by default
// Uses the ancient ncurses text windowing library
// Also services keyboard and tuning knob, if present
void *display(void *arg){
  struct demod * const demod = arg;
  double tunestep10 = pow(10.,Tunestep);
  int tuneitem = 0;

  assert(demod != NULL);
  pthread_setname_np(pthread_self(),"display");
  pthread_cleanup_push(display_cleanup,demod);

  initscr();
  keypad(stdscr,TRUE);
  timeout(Update_interval); // This sets the update interval when nothing is typed
  cbreak();
  noecho();
#if 0
  WINDOW * const deb = newwin(5,40,14,40);
  scrollok(deb,1);
#endif
  WINDOW * const fw = newwin(9,70,0,0);
  WINDOW * const sig = newwin(10,25,10,0);
  WINDOW * const sdr = newwin(7,30,10,25);
  WINDOW * const net = newwin(6,70,21,0);
  
  int const dial_fd = open(DIAL,O_RDONLY|O_NDELAY);

  for(;;){
    struct bandplan const *bp;
    
    // Update display
    // Frequency control section - these can be adjusted by the user
    // using the keyboard or tuning knob
    wmove(fw,0,0);
    wprintw(fw,"Frequency   %'17.3f Hz",get_freq(demod));
    if(demod->frequency_lock)
      wprintw(fw," LOCK");
    // Display ham band emission data, if available
    if((bp = lookup_frequency(get_freq(demod))) != NULL){
      wprintw(fw," %s",bp->name);
      if(bp->modes & CW)
	wprintw(fw," CW");
      if(bp->modes & DATA)
	wprintw(fw," Data");
      if(bp->modes & VOICE)
	wprintw(fw," Voice");
      if(bp->modes & IMAGE)
	wprintw(fw," Image");
      if(bp->power != 0)
	wprintw(fw," %.0lf watts",bp->power);
    }
    wprintw(fw,"\n");
      
    wprintw(fw,"First LO    %'17.3f Hz\n",get_first_LO(demod));
    wprintw(fw,"IF          %'17.3f Hz",-get_second_LO(demod,0));
    if(!LO2_in_range(demod,get_second_LO(demod,0),1)){
      double alias;
      if(get_second_LO(demod,0) > 0)
	alias = get_first_LO(demod) - get_second_LO(demod,0) + demod->samprate;
      else
	alias = get_first_LO(demod) - get_second_LO(demod,0) - demod->samprate;	
      wprintw(fw," alias %'.3f Hz",alias);
    }
    wprintw(fw,"\n");
    wprintw(fw,"Dial offset %'17.3f Hz\n",demod->dial_offset);
    wprintw(fw,"Calibrate   %'17.3f ppm\n",get_cal(demod)*1e6);
    wprintw(fw,"Filter low  %'17.3f Hz\n",demod->low);
    wprintw(fw,"Filter high %'17.3f Hz\n",demod->high);
    wprintw(fw,"Kaiser Beta %'17.3f\n",Kaiser_beta);
    wprintw(fw,"Spare       %'17.3f\n",Spare);
    // Highlight cursor for tuning step
    // A little messy because of the commas in the frequencies
    // They come from the ' option in the printf formats
    int x;

    if(Tunestep >= -3 && Tunestep <= -1){ // .001 or .01 or .1
      x = 24 - Tunestep + 1;
    } else if(Tunestep >= 0 && Tunestep <= 2){
      x = 24 - Tunestep;  // 1, 10, 100
    } else if(Tunestep >= 3 && Tunestep <= 5){
      x = 24 - Tunestep - 1; // 1,000; 10,000; 100,000
    } else if(Tunestep >= 6 && Tunestep <= 8){
      x = 24 - Tunestep - 2; // 1,000,000; 10,000,000; 100,000,000
    } else if(Tunestep >= 9 && Tunestep <= 9){
      x = 24 - Tunestep - 3; // 1,000,000,000
    } else
      x = 0; // can't happen, but shuts up compiler
    // Highlight digit
    mvwchgat(fw,tuneitem,x,1,A_STANDOUT,0,NULL);
    wnoutrefresh(fw);

    // Signal data: demod mode, filter block sizes and signal levels
    wclrtobot(sig);     // clear previous stuff in case we stop showing the last lines
    wmove(sig,0,0);
    wprintw(sig,"Mode        %s\n",Modes[demod->mode].name);
    wprintw(sig,"Blocksize   %d\n",demod->L);
    wprintw(sig,"Impulse len %d\n",demod->M);
    wprintw(sig,"IF      %7.1f dBFS\n",power2dB(demod->power_i + demod->power_q));
    wprintw(sig,"Baseband%7.1f dBFS\n",voltage2dB(demod->amplitude));
    wprintw(sig,"AF Gain %7.1f dB\n",voltage2dB(demod->gain));
#if 0
    // Ratio of baseband to IF power, adjusted for bandwidth ratios
    // Gives good SNR estimate **only** when there are no out-of-band signals
    // and the noise bandwidth of the filter is close to fabs(high-low), which is
    // probably not true for small bandwidths (should probably adjust for Kaiser beta)
    float n0 = ((demod->power_i + demod->power_q) - (demod->amplitude * demod->amplitude))
      / (demod->samprate - fabs(demod->high-demod->low));
#endif
    if(!isnan(demod->snr))
      wprintw(sig,"SNR     %7.1f dB\n",power2dB(demod->snr));

    if(!isnan(demod->foffset))
      wprintw(sig,"offset  %+7.1f Hz\n",demod->foffset);

    if(!isnan(demod->pdeviation))
      wprintw(sig,"deviat  %7.1f Hz\n",demod->pdeviation);

    if(!isnan(demod->cphase))
      wprintw(sig,"cphase  %7.3f rad\n",demod->cphase);

    if(!isnan(demod->plfreq))
      wprintw(sig,"plfreq  %7.1f Hz\n",demod->plfreq);

    wnoutrefresh(sig);
    
    // SDR front end hardware status, I/Q offset and imbalance information
    wmove(sdr,0,0);
    wprintw(sdr,"I offset %10.6f\n",demod->DC_i);
    wprintw(sdr,"Q offset %10.6f\n",demod->DC_q);
    wprintw(sdr,"I/Q imbal%10.3f dB\n",power2dB(demod->power_i/demod->power_q));
    double sinphi = demod->dotprod / (demod->power_i + demod->power_q);
    wprintw(sdr,"I/Q phi  %10.5f rad\n",sinphi);
    wprintw(sdr,"LNA      %10u\n",demod->lna_gain);
    wprintw(sdr,"Mix gain %10u\n",demod->mixer_gain);
    wprintw(sdr,"IF gain  %10u dB\n",demod->if_gain);
    wnoutrefresh(sdr);
    
    extern int Delayed,Skips;

    char source[INET6_ADDRSTRLEN];
    int sport=-1;

    inet_ntop(AF_INET,&Input_source_address.sin_addr,source,sizeof(source));
    sport = ntohs(Input_source_address.sin_port);

    // Multicast packet I/O information
    wmove(net,0,0);
    if(OPUS_bitrate > 0){
      wprintw(net,"OPUS audio -> %s:%d; %'d bps %.1f ms blocks\n",BB_mcast_address_text,Mcast_dest_port,
	      OPUS_bitrate,OPUS_blocktime);
    } else {
      wprintw(net,"PCM audio -> %s:%d\n",BB_mcast_address_text,Mcast_dest_port);
    }
    wprintw(net,"IQ in %s:%d -> %s:%d\n",source,sport,IQ_mcast_address_text,Mcast_dest_port);
    wprintw(net,"Delayed %d Skips %d\n",Delayed,Skips);

    wnoutrefresh(net);
    doupdate();


    // Poll Griffin Powermate knob, if present

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
      if(event.type == EV_SYN){
	// Ignore
      } if(event.type == EV_REL){
	if(event.code == REL_DIAL){
	  // Dial has been turned. Simulate up/down arrow tuning commands
	  if(event.value > 0){
	    adjust_item(demod,tuneitem,tunestep10);
	  } else if(event.value < 0)
	    adjust_item(demod,tuneitem,-tunestep10);
	}
      } else if(event.type == EV_KEY){
	if(event.code == BTN_MISC)
	  if(event.value)
	    demod->frequency_lock = !demod->frequency_lock; // Toggle frequency tuning lock
      }
      continue; // Start again with display refresh
    }

    int c;
    c = getch(); // read keyboard with timeout

    switch(c){
    case ERR:   // no key; timed out. Do nothing.
      break;
    case 'q':   // Exit radio program
      goto done;
    case 'h':
    case '?':
      popup("help.txt");
      break;
    case 'I':
      { // This seems to need work
	char str[160];
	getentry("IQ input IP dest address: ",str,sizeof(str));
	int const i = setup_input(str);
	int const j = Input_fd;
	Input_fd = i;
	if(j != -1)
	  close(j);
	if(IQ_mcast_address_text != NULL)
	  free(IQ_mcast_address_text);
	IQ_mcast_address_text = strdup(str);
      }
      break;
    case 'l': // Toggle RF tuning lock; affects how adjustments to LO and IF behave
      demod->frequency_lock = !demod->frequency_lock;
      break;
    case KEY_NPAGE: // Page Down key
    case '\t':      // tab: cycle forwards through tuning fields
      tuneitem = (tuneitem + 1) % 9;
      break;
    case KEY_BTAB:  // Backtab, i.e., shifted tab: cycle backwards through tuning fields
    case KEY_PPAGE: // Page Up
      tuneitem = (9 + tuneitem - 1) % 9;
      break;
    case KEY_HOME: // Go back to top row
      tuneitem = 0;
      Tunestep = 0;
      tunestep10 = 1;
      break;
    case KEY_BACKSPACE: // Cursor left: increase tuning step 10x
    case KEY_LEFT:
      if(Tunestep < 9){
	Tunestep++;
	tunestep10 *= 10;
      }
      break;
    case KEY_RIGHT:     // Cursor right: decrease tuning step /10
      if(Tunestep > -3){
	Tunestep--;
	tunestep10 /= 10;
      }
      break;
    case KEY_UP:        // Increase whatever we're tuning
      adjust_item(demod,tuneitem,tunestep10);
      break;
    case KEY_DOWN:      // Decrease whatever we're tuning
      adjust_item(demod,tuneitem,-tunestep10);
      break;
    case '\f':  // Screen repaint (formfeed, aka control-L)
      clearok(curscr,TRUE);
      break;
    case 'b':   // Blocksize - both data and impulse response-1
      {
	char str[160],*ptr;
	getentry("Enter blocksize in samples: ",str,sizeof(str));
	int const i = strtol(str,&ptr,0);
	if(ptr != str){
	  demod->L = i;
	  demod->M = demod->L + 1;
	  set_mode(demod,demod->mode); // Restart demod thread
	}
      }
      break;
    case 'c':   // TCXO calibration offset, also affects sampling clock
      {
	char str[160],*ptr;
	getentry("Enter calibrate offset in ppm: ",str,sizeof(str));
	double const f = strtod(str,&ptr);
	if(ptr != str)
	  set_cal(demod,f * 1e-6);
      }
      break;
    case 'n':   // Set noise reference to current amplitude; hit with no sig
      demod->noise = demod->amplitude;
      break;
    case 'm':   // Select demod mode from list
      {
	char str[160];
	strncpy(str,"Enter mode [ ",sizeof(str));
	int i;
	for(i=0;i < Nmodes;i++){
	  strncat(str,Modes[i].name,sizeof(str) - strlen(str) - 1);
	  strncat(str," ",sizeof(str) - strlen(str) - 1);
	}
	strncat(str,"]: ",sizeof(str) - strlen(str) - 1);
	getentry(str,str,sizeof(str));
	if(strlen(str) > 0){
	  for(i=0;i < Nmodes;i++){
	    if(strcasecmp(str,Modes[i].name) == 0){
	      set_mode(demod,Modes[i].mode);
	      break;
	    }
	  }
	}
      }
      break;
    case 'f':   // Tune to new frequency
      {
	char str[160];
	getentry("Enter frequency: ",str,sizeof(str));
	double const f = parse_frequency(str);
	if(f > 0)
	  set_freq(demod,f,0);
      }
      break;
    case 'i':    // Recenter IF
      set_freq(demod,get_freq(demod),1);
      break;
    case 'u': // Display update rate
      {
	char str[160],*ptr;
	getentry("Enter update interval, ms [<=0 means no auto update]: ",str,sizeof(str));
	int const u = strtol(str,&ptr,0);
	if(ptr != str){
	  if(u > 50){
	    Update_interval = u;
	    timeout(Update_interval);
	  } else if(u <= 0){
	    Update_interval = -1; // No automatic update
	    timeout(Update_interval);
	  }
	}
      }
      break;
    case 'k': // Kaiser window beta parameter
      {
	char str[160],*ptr;
	getentry("Enter Kaiser window beta: ",str,sizeof(str));
	double const b = strtod(str,&ptr);
	if(ptr != str && b >= 0 && b < 100 && b != Kaiser_beta){
	  Kaiser_beta = b;
	  set_filter(demod,demod->low,demod->high);
	}
      }
      break;
    default:
      //      fprintf(stderr,"char %d 0x%x",c,c);
      break;
    }
    doupdate();
  }
 done:;
  pthread_cleanup_pop(1);
  {
    // Dump receiver state to file
    char statefile[PATH_MAX];
    snprintf(statefile,sizeof(statefile),"%s/.radiostate",getenv("HOME"));
    savestate(demod,statefile);
  }
  exit(0);
}
