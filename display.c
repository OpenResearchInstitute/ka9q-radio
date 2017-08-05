// $Id: display.c,v 1.62 2017/08/04 14:57:38 karn Exp karn $
// Thread to display internal state of 'radio' and accept single-letter commands
// Copyright 2017 Phil Karn, KA9Q
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
#include <sys/time.h>
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

// Pop up a temporary window with the contents of a file in the
// library directory (usually /usr/local/share/ka9q-radio/)
// then wait for a single keyboard character to clear it

void popup(const char *filename){
  static const int maxcols = 256;
  char fname[PATH_MAX];
  snprintf(fname,sizeof(fname),"%s/%s",Libdir,filename);
  FILE *fp;
  if((fp = fopen(fname,"r")) == NULL)
    return;
  // Determine size of box
  int rows=0, cols=0;
  char line[maxcols];
  while(fgets(line,sizeof(line),fp) != NULL){
    chomp(line);
    rows++;
    if(strlen(line) > cols)
      cols = strlen(line); // Longest line
  }
  rewind(fp);
  
  // Allow room for box
  WINDOW * const pop = newwin(rows+2,cols+2,0,0);
  box(pop,0,0);
  int row = 1; // Start inside box
  while(fgets(line,sizeof(line),fp) != NULL){
    chomp(line);
    mvwprintw(pop,row++,1,line);
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
  struct audio * const audio = demod->audio;
  double tunestep10 = pow(10.,demod->tunestep);
  int tuneitem = 0;

  pthread_cleanup_push(display_cleanup,demod);

  initscr();
  keypad(stdscr,TRUE);
  timeout(Update_interval); // update interval when nothing is typed
  cbreak();
  noecho();

  WINDOW * const fw = newwin(9,70,0,0);    // Frequency information
  WINDOW * const sig = newwin(13,40,10,0); // Signal information
  WINDOW * const sdr = newwin(17,40,6,40); // SDR information
  WINDOW * const status = newwin(5,40,0,40); // Misc status information
  
  int const dial_fd = open(DIAL,O_RDONLY|O_NDELAY);  // Powermate knob?

  struct sockaddr old_input_source_address;
  char source[INET6_ADDRSTRLEN];
  char sport[256];
  memset(source,0,sizeof(source));
  memset(sport,0,sizeof(sport));

  struct timeval last_time;

  for(;;){

    // Update display
    // Frequency control section - these can be adjusted by the user
    // using the keyboard or tuning knob, so be careful with formatting
    wmove(fw,0,0);
    wclrtobot(fw);
    wprintw(fw,"Frequency   %'17.3f Hz",get_freq(demod));
    if(demod->frequency_lock)
      wprintw(fw," LOCK");

    wprintw(fw,"\n");
      
    // second LO frequency is negative of IF, i.e., a signal at +48 kHz
    // needs a second LO frequency of -48 kHz to bring it to zero
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
    int hcol;
    if(demod->tunestep >= -3 && demod->tunestep <= -1){ // .001 or .01 or .1
      hcol = 24 - demod->tunestep + 1;
    } else if(demod->tunestep >= 0 && demod->tunestep <= 2){
      hcol = 24 - demod->tunestep;  // 1, 10, 100
    } else if(demod->tunestep >= 3 && demod->tunestep <= 5){
      hcol = 24 - demod->tunestep - 1; // 1,000; 10,000; 100,000
    } else if(demod->tunestep >= 6 && demod->tunestep <= 8){
      hcol = 24 - demod->tunestep - 2; // 1,000,000; 10,000,000; 100,000,000
    } else if(demod->tunestep >= 9 && demod->tunestep <= 9){
      hcol = 24 - demod->tunestep - 3; // 1,000,000,000
    } else
      hcol = 0; // can't happen, but shuts up compiler
    // Highlight digit
    mvwchgat(fw,tuneitem,hcol,1,A_STANDOUT,0,NULL);
    wnoutrefresh(fw);


    wmove(status,0,0);
    wclrtobot(status);     // clear previous stuff in case we stop showing the last lines
    // Display ham band emission data, if available
    struct bandplan const *bp_low,*bp_high;
    bp_low = lookup_frequency(get_freq(demod)+demod->low);
    bp_high = lookup_frequency(get_freq(demod)+demod->high);
    // Make sure entire receiver passband is in the band
    if(bp_low != NULL && bp_high != NULL){
      struct bandplan intersect;

      // If the passband straddles a mode or class boundary, form
      // the intersection to give the more restrictive answers
      intersect.classes = bp_low->classes & bp_high->classes;
      intersect.modes = bp_low->modes & bp_high->modes;

      wprintw(status,"%s\n",bp_low->name); // bp_high->name should be the same
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

      // Any power limits (except for Novices?)
      if(bp_low->power != 0)
	wprintw(status,"Limit %.0lf watts\n",bp_low->power);
      else if(bp_high->power != 0)
	wprintw(status,"Limit %.0lf watts\n",bp_high->power);
    }
    if(!LO2_in_range(demod,get_second_LO(demod),1)){
      // LO2 is near its edges where signals from the opposite edge
      // get aliased; warn about this
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
    // Display these only if they're in use by the current mode
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
    wprintw(sdr,"Src   %s:%s\n",source,sport); // Actual sender of mcasts
    wprintw(sdr,"Mcast %s:%s\n",demod->iq_mcast_address_text,Mcast_dest_port);
    wprintw(sdr,"IQ Pkts%'14lld\n",demod->iq_packets);
    wprintw(sdr,"Late%17d\n",Delayed);
    wprintw(sdr,"Skips%16d\n",Skips);
    wprintw(sdr,"Samprate%'13d Hz\n",demod->status.samprate); // Nominal
    wprintw(sdr,"I offset%13.6f\n",demod->DC_i);  // Scaled to +/-1
    wprintw(sdr,"Q offset%13.6f\n",demod->DC_q);
    wprintw(sdr,"I/Q imbal%12.3f dB\n",power2dB(demod->power_i/demod->power_q));
    wprintw(sdr,"I/Q phi%14.5f rad\n",demod->sinphi);
    wprintw(sdr,"LNA%18u\n",demod->status.lna_gain);   // SDR dependent
    wprintw(sdr,"Mix gain%13u\n",demod->status.mixer_gain); // SDR dependent
    wprintw(sdr,"IF gain%14u dB\n",demod->status.if_gain); // SDR dependent
    // Audio output dest address (usually multicast)
    wprintw(sdr,"Dest  %s:%s\n",audio->audio_mcast_address_text,Mcast_dest_port);
    if(audio->opus_bitrate > 0){
      wprintw(sdr,"Opus%3.0fms %s%'8.0f bps\n",
	      audio->opus_blocktime,
	      audio->opus_dtx ? "dtx":"   ",

	      (float)audio->opus_bitrate);
    } else // Actual data rate depends on mono/stereo
           // 16xDAC_samprate for mono, 32xDAC_samprate for stereo
      wprintw(sdr,"PCM%'16.0f bps\n",(float)audio->samprate);
    struct timeval tv;
    gettimeofday(&tv,0);
    long long dt = 1000000 * (tv.tv_sec - last_time.tv_sec) +
      tv.tv_usec - last_time.tv_usec;
    last_time = tv;
    float decay;
    decay = exp(-0.5 * dt/1.0e6);
    audio->bitrate *= decay;

      
    wprintw(sdr,"traffic%'14.0f bps\n",audio->bitrate);
    audio->bitrate *= decay;

    wprintw(sdr,"Pkts%'17lld\n",audio->audio_packets);

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

    // Scan and process keyboard commands
    int c = getch(); // read keyboard with timeout

    switch(c){
    case ERR:   // no key; timed out. Do nothing.
      break;
    case 'q':   // Exit entire radio program
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
	// demod->input_fd is not protected by a mutex, so swap it carefully
	// Mutex protection would be difficult because input thread is usually
	// blocked on the socket, and if there's no I/Q input we'd hang
	int const j = demod->input_fd;
	demod->input_fd = i;
	if(j != -1)
	  close(j); // This should cause the input thread to see an error
	strncpy(demod->iq_mcast_address_text,str,sizeof(demod->iq_mcast_address_text));
	// Reset error counts
	Skips = Delayed = 0;
      }
      break;
    case 'l': // Toggle RF tuning lock; affects how adjustments to LO and IF behave
      demod->frequency_lock = !demod->frequency_lock;
      break;
    case KEY_NPAGE: // Page Down/tab key
    case '\t':      // go to next tuning item
      tuneitem = (tuneitem + 1) % 9;
      break;
    case KEY_BTAB:  // Page Up/Backtab, i.e., shifted tab:
    case KEY_PPAGE: // go to previous tuning item
      tuneitem = (9 + tuneitem - 1) % 9;
      break;
    case KEY_HOME: // Go back to item 0
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
    case KEY_UP:        // Increase whatever digit we're tuning
      adjust_item(demod,tuneitem,tunestep10);
      break;
    case KEY_DOWN:      // Decrease whatever we're tuning
      adjust_item(demod,tuneitem,-tunestep10);
      break;
    case '\f':  // Screen repaint (formfeed, aka control-L)
      clearok(curscr,TRUE);
      break;
    case 'b':   // Blocksize - sets both data and impulse response-1
                // They should be separably set. Do this in the state file for now
      {
	char str[160],*ptr;
	getentry("Enter blocksize in samples: ",str,sizeof(str));
	int const i = strtol(str,&ptr,0);
	if(ptr != str){
	  demod->L = i;
	  demod->M = demod->L + 1;
	  set_mode(demod,demod->mode,0); // Restart demod thread
	}
      }
      break;
    case 'c':   // TCXO calibration offset, also affects sampling clock
      {
	char str[160],*ptr;
	getentry("Enter calibration offset in ppm: ",str,sizeof(str));
	double const f = strtod(str,&ptr);
	if(ptr != str)
	  set_cal(demod,f * 1e-6);
      }
      break;
    case 'n':   // Set noise reference to current amplitude; hit with no sig
      demod->noise = demod->amplitude;
      break;
    case 'm':   // Select demod mode from list. Note: overwrites filters
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
	      set_mode(demod,Modes[i].mode,1); // With default filters
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
	double const f = parse_frequency(str); // Handles funky forms like 147m435
	if(f > 0){
	  // If frequency would be out of range, guess kHz or MHz
	  if(f >= 0.1 && f < 100)
	    set_freq(demod,f*1e6,1); // 0.1 - 99.999 Only MHz can be valid
	  else if(f < 500)         // 100-499.999 could be kHz or MHz, assume MHz
	    set_freq(demod,f*1e6,1);
	  else if(f < 2000)        // 500-1999.999 could be kHz or MHz, assume kHz
	    set_freq(demod,f*1e3,1);
	  else if(f < 100000)      // 2000-99999.999 can only be kHz
	    set_freq(demod,f*1e3,1);
	  else                     // accept directly
	    set_freq(demod,f,1); 
	}
      }
      break;
    case 'i':    // Recenter IF to +/- samprate/4
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
