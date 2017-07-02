// $Id: display.c,v 1.44 2017/06/28 04:35:47 karn Exp karn $
// Thread to display internal state of 'radio' and accept single-letter commands
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

int Update_interval = 100;

// Parse a frequency entry in the form
// 12345 (12345 Hz)
// 12k345 (12.345 kHz)
// 12m345 (12.345 MHz)
// 12g345 (12.345 GHz)

const double parse_frequency(const char *s){
  char *ss = alloca(strlen(s));

  int i;
  for(i=0;i<strlen(s);i++)
    ss[i] = tolower(s[i]);

  ss[i] = '\0';
  
  char *sp;
  double mult = 1.0;
  if((sp = strchr(ss,'g')) != NULL){
    mult = 1e9;
    *sp = '.';
  } else if((sp = strchr(ss,'m')) != NULL){
    mult = 1e6;
    *sp = '.';
  } else if((sp = strchr(ss,'k')) != NULL){
    mult = 1e3;
    *sp = '.';
  } else
    mult = 1;

  char *endptr;
  double f = strtod(ss,&endptr);
  if(endptr != ss)
    return mult * f;
  else
    return 0;
}


void chomp(char *s){
  char *cp;

  if(s == NULL)
    return;
  if((cp = strchr(s,'\r')) != NULL)
    *cp = '\0';
  if((cp = strchr(s,'\n')) != NULL)
    *cp = '\0';
}


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
  WINDOW *pop = newwin(rows+2,cols+2,0,0);
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
  (void)getch();
  timeout(Update_interval);
  werase(pop);
  wrefresh(pop);
  delwin(pop);
}


// Pop up a dialog box, issue a prompt and get a response
void getentry(char *prompt,char *response,int len){
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

static void adjust_item(struct demod *demod,const int tuneitem,const double tunestep){

  float low,high;

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
    ; // because next line is declaration
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
    get_filter(demod,&low,&high);
    low += tunestep;
    set_filter(demod,low,high);
    break;
  case 6: // Filter high edge
    get_filter(demod,&low,&high);
    high += tunestep;
    set_filter(demod,low,high);
    break;
  case 7: // Kaiser beta
    get_filter(demod,&low,&high);
    if(Kaiser_beta + tunestep >= 0.0){
      Kaiser_beta += tunestep;
      set_filter(demod,low,high);
    }
    break;
  }
}

void *display(void *arg){
  int c;
  struct demod *demod = &Demod;
  int tunestep = 0;
  double tunestep10 = 1;
  int tuneitem = 0;
  int dial_fd;
  WINDOW *fw,*sig,*sdr,*net;

  pthread_cleanup_push(display_cleanup,demod);
  pthread_setname_np(pthread_self(),"display");

  initscr();
  keypad(stdscr,TRUE);
  timeout(Update_interval); // This sets the update interval when nothing is typed
  cbreak();
  noecho();
#if 0
  deb = newwin(5,40,14,40);
  scrollok(deb,1);
#endif
  fw = newwin(9,70,0,0);
  sig = newwin(7,25,10,0);
  sdr = newwin(7,30,10,25);
  net = newwin(6,70,18,0);
  
  dial_fd = open(DIAL,O_RDONLY|O_NDELAY);


  for(;;){
    float low,high;
    const struct bandplan *bp;
    
    // Update display
    wmove(fw,0,0);
    get_filter(demod,&low,&high);
    wprintw(fw,"Frequency   %'17.2f Hz",get_freq(demod));
    if(demod->frequency_lock)
      wprintw(fw," LOCK");
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
      
    wprintw(fw,"First LO    %'17.2f Hz\n",get_first_LO(demod));
    wprintw(fw,"IF          %'17.2f Hz",-get_second_LO(demod,0));
    if(!LO2_in_range(demod,get_second_LO(demod,0),1)){
      double alias;
      if(get_second_LO(demod,0) > 0)
	alias = get_first_LO(demod) - get_second_LO(demod,0) + demod->samprate;
      else
	alias = get_first_LO(demod) - get_second_LO(demod,0) - demod->samprate;	
      wprintw(fw," alias %'.2f Hz",alias);
    }
    wprintw(fw,"\n");
    wprintw(fw,"Dial offset %'17.2f Hz\n",demod->dial_offset);
    wprintw(fw,"Calibrate   %'17.2f ppm\n",get_cal(demod)*1e6);
    wprintw(fw,"Filter low  %'17.2f Hz\n",low);
    wprintw(fw,"Filter high %'17.2f Hz\n",high);
    wprintw(fw,"Kaiser Beta %'17.2f\n",Kaiser_beta);
    wprintw(fw,"Blocksize   %17d\n",demod->L);
    // Tuning step highlight
    // A little messy because of the commas in the frequencies
    // They come from the ' option in the printf formats
    int x;

    if(tunestep >= -2 && tunestep <= -1){ // .01 or .1
      x = 25 - tunestep + 1;
    } else if(tunestep >= 0 && tunestep <= 2){
      x = 25 - tunestep;  // 1, 10, 100
    } else if(tunestep >= 3 && tunestep <= 5){
      x = 25 - tunestep - 1; // 1,000; 10,000; 100,000
    } else if(tunestep >= 6 && tunestep <= 8){
      x = 25 - tunestep - 2; // 1,000,000; 10,000,000; 100,000,000
    } else if(tunestep >= 9 && tunestep <= 9){
      x = 25 - tunestep - 3; // 1,000,000,000
    } else
      x = 0; // can't happen, but shuts up compiler
    // Highlight digit for current tuning step
    mvwchgat(fw,tuneitem,x,1,A_STANDOUT,0,NULL);
    wnoutrefresh(fw);

    wclrtobot(sig);     // clear previous stuff in case we stop showing the last lines
    wmove(sig,0,0);

    wprintw(sig,"Mode         %3s\n",Modes[demod->mode].name);
    wprintw(sig,"IF      %7.1f dBFS\n",power2dB(demod->power_i + demod->power_q));
    wprintw(sig,"Baseband%7.1f dBFS\n",voltage2dB(demod->amplitude));
    wprintw(sig,"AF Gain %7.1f dB\n",voltage2dB(demod->gain));
#if 0
    // Ratio of baseband to IF power, adjusted for bandwidth ratios
    // Gives good SNR estimate **only** when there are no out-of-band signals
    // and the noise bandwidth of the filter is close to fabs(high-low), which is
    // probably not true for small bandwidths (should probably adjust for Kaiser beta)
    float n0 = ((demod->power_i + demod->power_q) - (demod->amplitude * demod->amplitude))
      / (demod->samprate - fabs(high-low));
#endif
    if(!isnan(demod->snr))
      wprintw(sig,"SNR     %7.1f dB\n",power2dB(demod->snr));
    if(!isnan(demod->foffset))
      wprintw(sig,"offset  %+7.1f Hz\n",demod->foffset);

    if(!isnan(demod->pdeviation))
      wprintw(sig,"deviat  %7.1f Hz\n",demod->pdeviation);

    if(!isnan(demod->cphase))
      wprintw(sig,"cphase  %7.3f rad\n",demod->cphase);

    wnoutrefresh(sig);
    
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

    wmove(net,0,0);
    wprintw(net,"IQ in %s:%d -> %s:%d\n",source,sport,IQ_mcast_address_text,Mcast_dest_port);
    wprintw(net,"Delayed %d\n",Delayed);
    wprintw(net,"Skips   %d\n",Skips);

    if(OPUS_bitrate > 0){
      wprintw(net,"OPUS audio -> %s:%d; %'d bps %.1f ms blocks\n",BB_mcast_address_text,Mcast_dest_port,
	      OPUS_bitrate,OPUS_blocktime);

    } else {
      wprintw(net,"PCM audio -> %s:%d\n",BB_mcast_address_text,Mcast_dest_port);
    }
    wnoutrefresh(net);
    doupdate();


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

    // Poll Powermate knob
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
      {
	char str[160];
	getentry("IQ input IP dest address: ",str,sizeof(str));
	int i = setup_input(str);
	int j = Input_fd;
	Input_fd = i;
	if(j != -1)
	  close(j);
	if(IQ_mcast_address_text != NULL)
	  free(IQ_mcast_address_text);
	IQ_mcast_address_text = strdup(str);
      }
      break;
    case 'l':
      demod->frequency_lock = !demod->frequency_lock;
      break;
    case KEY_NPAGE: // Page Down
    case '\t':  // tab: cycle through tuning fields
      tuneitem = (tuneitem + 1) % 8;
      break;
    case KEY_BTAB: // Backtab, i.e., shifted tab: cycle backwards through tuning fields
    case KEY_PPAGE: // Page Up
      tuneitem = (8 + tuneitem - 1) % 8;
      break;
    case KEY_HOME: // Go back to starting spot
      tuneitem = 0;
      tunestep = 0;
      tunestep10 = 1;
      break;
    case KEY_BACKSPACE: // Cursor left: increase tuning step 10x
    case KEY_LEFT:
      if(tunestep < 9){
	tunestep++;
	tunestep10 *= 10;
      }
      break;
    case KEY_RIGHT:     // Cursor right: decrease tuning step /10
      if(tunestep > -2){
	tunestep--;
	tunestep10 /= 10;
      }
      break;
    case KEY_UP:        // Increase whatever we're tuning
      adjust_item(demod,tuneitem,tunestep10);
      break;
    case KEY_DOWN:      // Decrease whatever we're tuning
      adjust_item(demod,tuneitem,-tunestep10);
      break;
    case '\f':  // Screen repaint
      clearok(curscr,TRUE);
      break;
    case 'b':   // Blocksize - both data and impulse response-1
      {
	char str[160];
	char *ptr;
	getentry("Enter blocksize in samples: ",str,sizeof(str));
	int i = strtol(str,&ptr,0);
	if(ptr != str){
	  demod->L = i;
	  demod->M = demod->L + 1;
	  set_mode(demod,demod->mode); // Restart demod thread
	}
      }
      break;
    case 'c':   // TCXO calibration offset
      {
	char str[160],*ptr;
	getentry("Enter calibrate offset in ppm: ",str,sizeof(str));
	double f = strtod(str,&ptr);
	if(ptr != str){
	  set_cal(demod,f * 1e-6);
	}
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
	double f = parse_frequency(str);
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
	int u = strtol(str,&ptr,0);
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
	double b = strtod(str,&ptr);
	  if(ptr != str && b >= 0 && b < 100 && b != Kaiser_beta){
	    Kaiser_beta = b;
	    get_filter(demod,&low,&high);
	    set_filter(demod,low,high);
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
  exit(0);
  pthread_exit(NULL);
}
