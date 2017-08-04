// $Id: display.c,v 1.61 2017/08/04 03:35:55 karn Exp karn $
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
#include <sys/socket.h>
#include <netdb.h>
// We need some of these definitions for the Griffin dial, but some conflict
// with definitions in ncurses.h !!arrrrggggh!!
// So we'll declare ourselves the small parts we need later
//#include <linux/input.h>

#include "rtp.h"
#include "radio.h"
#include "audio.h"
#include "dsp.h"
#include "filter.h"
#include "multicast.h"
#include "bandplan.h"

#define DIAL "/dev/input/by-id/usb-Griffin_Technology__Inc._Griffin_PowerMate-event-if00"

float Spare; // General purpose knob for experiments

extern int Update_interval;

// Pop up a temporary window with the contents of a file,
// then wait for a kb character to clear it
#define MAXCOLS 256
void popup(const char *filename){

  char fname[PATH_MAX];
  snprintf(fname,sizeof(fname),"%s/%s",Libdir,filename);
  FILE *fp;
  if((fp = fopen(fname,"r")) == NULL)
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
      double new_lo2 = get_second_LO(demod) + tunestep * (1 + demod->calibrate);
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
    double new_lo2 = get_second_LO(demod);

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
    set_cal(demod,demod->calibrate + tunestep * 1e-6); // ppm
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
    if(demod->kaiser_beta + tunestep >= 0.0){
      demod->kaiser_beta += tunestep;
      set_filter(demod,demod->low,demod->high); // Recreate filters
    }
    break;
  case 8: // Spare for experimentation
    if(Spare + tunestep > -demod->samprate/2 && Spare + tunestep < demod->samprate/2){
      Spare += tunestep;
      // Experimental notch filter
      struct notchfilter *nf = notch_create(Spare/demod->samprate,.001);
      struct notchfilter *old_nf = demod->nf;
      demod->nf = nf;
      if(old_nf)
	notch_delete(old_nf);
      break;
    }
  }
}

// Thread to display receiver state, updated at 10Hz by default
// Uses the ancient ncurses text windowing library
// Also services keyboard and tuning knob, if present
void *display(void *arg){
  pthread_setname("display");
  assert(arg != NULL);
  struct demod * const demod = arg;
  double tunestep10 = pow(10.,demod->tunestep);
  int tuneitem = 0;

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
  WINDOW * const sig = newwin(13,40,10,0);
  WINDOW * const sdr = newwin(16,40,6,40);
  WINDOW * const status = newwin(5,40,0,40);
  
  int const dial_fd = open(DIAL,O_RDONLY|O_NDELAY);

  struct sockaddr old_input_source_address;
  char source[INET6_ADDRSTRLEN];
  char sport[256];
  source[0] = 0;
  sport[0] = 0;

  for(;;){

    
    // Update display
    // Frequency control section - these can be adjusted by the user
    // using the keyboard or tuning knob
    wmove(fw,0,0);
    wclrtobot(fw);
    wprintw(fw,"Frequency   %'17.3f Hz",get_freq(demod));
    if(demod->frequency_lock)
      wprintw(fw," LOCK");

    wprintw(fw,"\n");
      
    wprintw(fw,"First LO    %'17.3f Hz\n",get_first_LO(demod));
    wprintw(fw,"IF          %'17.3f Hz\n",-get_second_LO(demod));
    wprintw(fw,"Dial offset %'17.3f Hz\n",demod->dial_offset);
    wprintw(fw,"Calibrate   %'17.3f ppm\n",demod->calibrate *1e6);
    wprintw(fw,"Filter low  %'17.3f Hz\n",demod->low);
    wprintw(fw,"Filter high %'17.3f Hz\n",demod->high);
    wprintw(fw,"Kaiser Beta %'17.3f\n",demod->kaiser_beta);
    wprintw(fw,"Spare       %'17.3f\n",Spare);
    // Highlight cursor for tuning step
    // A little messy because of the commas in the frequencies
    // They come from the ' option in the printf formats
    int x;

    if(demod->tunestep >= -3 && demod->tunestep <= -1){ // .001 or .01 or .1
      x = 24 - demod->tunestep + 1;
    } else if(demod->tunestep >= 0 && demod->tunestep <= 2){
      x = 24 - demod->tunestep;  // 1, 10, 100
    } else if(demod->tunestep >= 3 && demod->tunestep <= 5){
      x = 24 - demod->tunestep - 1; // 1,000; 10,000; 100,000
    } else if(demod->tunestep >= 6 && demod->tunestep <= 8){
      x = 24 - demod->tunestep - 2; // 1,000,000; 10,000,000; 100,000,000
    } else if(demod->tunestep >= 9 && demod->tunestep <= 9){
      x = 24 - demod->tunestep - 3; // 1,000,000,000
    } else
      x = 0; // can't happen, but shuts up compiler
    // Highlight digit
    mvwchgat(fw,tuneitem,x,1,A_STANDOUT,0,NULL);
    wnoutrefresh(fw);


    wmove(status,0,0);
    wclrtobot(status);     // clear previous stuff in case we stop showing the last lines
    // Display ham band emission data, if available
    struct bandplan const *bp_low,*bp_high;
    bp_low = lookup_frequency(get_freq(demod)+demod->low);
    bp_high = lookup_frequency(get_freq(demod)+demod->high);
    if(bp_low != NULL && bp_high != NULL){
      struct bandplan intersect;

      intersect.classes = bp_low->classes & bp_high->classes;
      intersect.modes = bp_low->modes & bp_high->modes;

      wprintw(status,"%s\n",bp_low->name); // Should be the same
      if(intersect.modes & CW)
	wprintw(status,"CW ");
      if(intersect.modes & DATA)
	wprintw(status,"Data ");
      if(intersect.modes & VOICE)
	wprintw(status,"Voice ");
      if(intersect.modes & IMAGE)
	wprintw(status,"Image ");
      wprintw(status,"\n");
      if(intersect.classes & EXTRA_CLASS)
	wprintw(status,"Extra ");
      if(intersect.classes & ADVANCED_CLASS)
	wprintw(status,"Advanced ");
      if(intersect.classes & GENERAL_CLASS)
	wprintw(status,"General ");
      if(intersect.classes & TECHNICIAN_CLASS)
	wprintw(status,"Technician ");
      if(intersect.classes & NOVICE_CLASS)
	wprintw(status,"Novice ");
      wprintw(status,"\n");


      if(bp_low->power != 0)
	wprintw(status,"Limit %.0lf watts\n",bp_low->power);
      else if(bp_high->power != 0)
	wprintw(status,"Limit %.0lf watts\n",bp_high->power);
    }
    if(!LO2_in_range(demod,get_second_LO(demod),1)){
      double alias;
      if(get_second_LO(demod) > 0)
	alias = get_first_LO(demod) - get_second_LO(demod) + demod->samprate;
      else
	alias = get_first_LO(demod) - get_second_LO(demod) - demod->samprate;	
      wprintw(status,"alias %'.3f Hz\n",alias);
    }

    wnoutrefresh(status);


    // Signal data: demod mode, filter block sizes and signal levels
    wmove(sig,0,0);
    wclrtobot(sig);     // clear previous stuff in case we stop showing the last lines
    wprintw(sig,"Mode    %10s\n",Modes[demod->mode].name);
    wprintw(sig,"Block   %'10d samp\n",demod->L);
    wprintw(sig,"Impulse %'10d samp\n",demod->M);
    wprintw(sig,"IF      %10.1f dBFS\n",power2dB(demod->power_i + demod->power_q));
    wprintw(sig,"Baseband%10.1f dBFS\n",voltage2dB(demod->amplitude));
    wprintw(sig,"AF Gain %10.1f dB\n",voltage2dB(demod->gain));
#if 0
    // Ratio of baseband to IF power, adjusted for bandwidth ratios
    // Gives good SNR estimate **only** when there are no out-of-band signals
    // and the noise bandwidth of the filter is close to fabs(high-low), which is
    // probably not true for small bandwidths (should probably adjust for Kaiser beta)
    float n0 = ((demod->power_i + demod->power_q) - (demod->amplitude * demod->amplitude))
      / (demod->samprate - fabs(demod->high-demod->low));
#endif
    if(!isnan(demod->snr))
      wprintw(sig,"SNR     %10.1f dB\n",power2dB(demod->snr));

    if(!isnan(demod->foffset))
      wprintw(sig,"offset  %+10.1f Hz\n",demod->foffset);

    if(!isnan(demod->pdeviation))
      wprintw(sig,"deviat  %10.1f Hz\n",demod->pdeviation);

    if(!isnan(demod->cphase))
      wprintw(sig,"cphase  %10.3f rad\n",demod->cphase);

    if(!isnan(demod->plfreq))
      wprintw(sig,"plfreq  %10.1f Hz\n",demod->plfreq);

    wnoutrefresh(sig);
    
    // SDR front end hardware status, I/Q offset and imbalance information
    if(memcmp(&old_input_source_address,&demod->input_source_address,sizeof(old_input_source_address)) != 0){
      // First time, or source has changed
      memcpy(&old_input_source_address,&demod->input_source_address,sizeof(old_input_source_address));
      getnameinfo((struct sockaddr *)&demod->input_source_address,sizeof(demod->input_source_address),
		  source,sizeof(source),
		  sport,sizeof(sport),NI_NOFQDN|NI_DGRAM);
    }

    wmove(sdr,0,0);
    wclrtobot(sdr);
    extern int Delayed,Skips;
    wprintw(sdr,"Src   %s:%s\n",source,sport);
    wprintw(sdr,"Mcast %s:%s\n",demod->iq_mcast_address_text,Mcast_dest_port);
    wprintw(sdr,"Late     %10d\n",Delayed);
    wprintw(sdr,"Skips    %10d\n",Skips);
    wprintw(sdr,"Samprate %'10d Hz\n",demod->status.samprate);
    wprintw(sdr,"I offset %10.6f\n",demod->DC_i);
    wprintw(sdr,"Q offset %10.6f\n",demod->DC_q);
    wprintw(sdr,"I/Q imbal%10.3f dB\n",power2dB(demod->power_i/demod->power_q));
    wprintw(sdr,"I/Q phi  %10.5f rad\n",demod->sinphi);
    wprintw(sdr,"LNA      %10u\n",demod->status.lna_gain);
    wprintw(sdr,"Mix gain %10u\n",demod->status.mixer_gain);
    wprintw(sdr,"IF gain  %10u dB\n",demod->status.if_gain);
    wprintw(sdr,"Dest  %s:%s\n",demod->audio->audio_mcast_address_text,Mcast_dest_port);
    if(demod->audio->opus_bitrate > 0){
      wprintw(sdr,"Opus %3.0f ms %'7d bps\n",demod->audio->opus_blocktime,demod->audio->opus_bitrate);
    } else
      wprintw(sdr,"PCM         %'7d Hz\n",DAC_samprate);

    wnoutrefresh(sdr);
    
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

    int c = getch(); // read keyboard with timeout

    switch(c){
    case ERR:   // no key; timed out. Do nothing.
      break;
    case 'q':   // Exit radio program
      goto done;
    case 'h':
    case '?':
      popup("help.txt");
      break;
    case 'w':
      {
	char str[160];
	getentry("Save state file: ",str,sizeof(str));
	if(strlen(str) > 0)
	  savestate(demod,str);
      }
      break;
    case 'I':
      {
	char str[160];
	getentry("IQ input IP dest address: ",str,sizeof(str));
	if(strlen(str) <= 0)
	  break;

	int const i = setup_mcast(str,Mcast_dest_port,0);
	if(i == -1){
	  beep();
	  break;
	}
	int const j = demod->input_fd;
	demod->input_fd = i;
	if(j != -1)
	  close(j);
	strncpy(demod->iq_mcast_address_text,str,sizeof(demod->iq_mcast_address_text));
	// Reset error counts
	Skips = Delayed = 0;
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
      demod->tunestep = 0;
      tunestep10 = 1;
      break;
    case KEY_BACKSPACE: // Cursor left: increase tuning step 10x
    case KEY_LEFT:
      if(demod->tunestep < 9){
	demod->tunestep++;
	tunestep10 *= 10;
      } else
	beep();
      break;
    case KEY_RIGHT:     // Cursor right: decrease tuning step /10
      if(demod->tunestep > -3){
	demod->tunestep--;
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
	  if(i == Nmodes)
	    beep();
	}
      }
      break;
    case 'f':   // Tune to new frequency
      {
	char str[160];
	getentry("Enter frequency: ",str,sizeof(str));
	double const f = parse_frequency(str);
	if(f > 0){
	  // If frequency would be out of range, guess kHz or MHz
	  if(f >= 0.1 && f < 100)
	    set_freq(demod,f*1e6,1); // 0.1 - 99.999 Only MHz can be valid
	  else if(f < 500)         // Could be kHz or MHz, arbitrarily assume MHz
	    set_freq(demod,f*1e6,1);
	  else if(f < 2000)        // Could be kHz or MHz, arbitarily assume kHz
	    set_freq(demod,f*1e3,1);
	  else if(f < 100000)      // Can only be kHz
	    set_freq(demod,f*1e3,1);
	  else                     // accept directly
	    set_freq(demod,f,1); 
	}
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
	  } else
	    beep();
	}
      }
      break;
    case 'k': // Kaiser window beta parameter
      {
	char str[160],*ptr;
	getentry("Enter Kaiser window beta: ",str,sizeof(str));
	double const b = strtod(str,&ptr);
	if(ptr != str && b >= 0 && b < 100 && b != demod->kaiser_beta){
	  demod->kaiser_beta = b;
	  set_filter(demod,demod->low,demod->high);
	} else
	  beep();
      }
      break;
    default:
      //      fprintf(stderr,"char %d 0x%x",c,c);
      beep();
      break;
    }
    doupdate();
  }
 done:;
  pthread_cleanup_pop(1);
  // Dump receiver state to default
  savestate(demod,"default");
  exit(0);
}
