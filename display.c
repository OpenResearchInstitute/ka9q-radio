// $Id: display.c,v 1.147 2018/12/11 10:57:10 karn Exp karn $
// Thread to display internal state of 'radio' and accept single-letter commands
// Why are user interfaces always the biggest, ugliest and buggiest part of any program?
// Copyright 2017 Phil Karn, KA9Q

#define _GNU_SOURCE 1
#include <assert.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <string.h>
#if defined(linux)
#include <bsd/string.h>
#endif
#include <math.h>
#include <complex.h>
#undef I
#include <sys/time.h>
#include <ncurses.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netdb.h>

#include "misc.h"
#include "dsp.h"
#include "radio.h"
#include "filter.h"
#include "multicast.h"
#include "bandplan.h"

// Control code to compute actual I/Q sample rate
// Doesn't work that well because NTP steers the clock of the host computer
#undef ACTUAL_SAMPLE_RATE


float Spare; // General purpose knob for experiments

// Touch screen position (Raspberry Pi display only - experimental)
int touch_x,touch_y;

extern int Update_interval;

// Screen location of field modification cursor
int mod_x,mod_y;



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
    mvwaddstr(pop,row++,1,line);
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
  WINDOW *pwin = newwin(5,90,15,0);
  box(pwin,0,0);
  mvwaddstr(pwin,1,1,prompt);
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

static FILE *Tty;
static SCREEN *Term;

void display_cleanup(void){
  echo();
  nocbreak();
  endwin();
  if(Term)
    delscreen(Term);
  Term = NULL;
  if(Tty)
    fclose(Tty);
  Tty = NULL;
}

static int Frequency_lock;

// Adjust the selected item up or down one step
void adjust_item(struct demod *demod,int direction){
  double tunestep;
  double samptime = demod->filter.decimate / (double)demod->input.samprate;
  
  tunestep = pow(10., (double)demod->tune.step);

  if(!direction)
    tunestep = - tunestep;

  switch(demod->tune.item){
  case 0: // Carrier frequency
  case 1: // Center frequency - treat the same
    if(!Frequency_lock) // Ignore if locked
      set_freq(demod,get_freq(demod) + tunestep,NAN);
    break;
  case 2: // First LO
    if(demod->tune.lock) // Tuner is locked, don't change it
      break;

    // Keep frequency but move LO2, which will move LO1 (if it can move)
    double new_lo2 = get_second_LO(demod) + tunestep;
    if(LO2_in_range(demod,new_lo2,0))
      set_freq(demod,get_freq(demod),new_lo2);
    break;
  case 3: // IF
    {
      double new_lo2 = get_second_LO(demod) - tunestep;
      if(LO2_in_range(demod,new_lo2,0)){ // Ignore if out of range
	// Vary RF and LO2 (IF) together to keep LO1 the same
	set_freq(demod,get_freq(demod) + tunestep,new_lo2);
      }
    }
    break;
  case 4: // Filter low edge
    demod->filter.low += tunestep;
    set_filter(demod->filter.out,samptime*demod->filter.low,samptime*demod->filter.high,demod->filter.kaiser_beta);
    break;
  case 5: // Filter high edge
    demod->filter.high += tunestep;
    set_filter(demod->filter.out,samptime*demod->filter.low,samptime*demod->filter.high,demod->filter.kaiser_beta);
    break;
  case 6: // Post-detection audio frequency shift
    demod->tune.shift += tunestep;
    set_shift(demod,demod->tune.shift);
    break;
  case 7: // Kaiser window beta parameter for filter
    demod->filter.kaiser_beta += tunestep;
    if(demod->filter.kaiser_beta < 0)
      demod->filter.kaiser_beta = 0;
    set_filter(demod->filter.out,samptime*demod->filter.low,samptime*demod->filter.high,demod->filter.kaiser_beta);
    break;
  }
}
// Hooks for knob.c (experimental)
// It seems better to just use the Griffin application to turn knob events into keystrokes or mouse events
void adjust_up(void *arg){
  struct demod *demod = arg;
  adjust_item(demod,1);
}
void adjust_down(void *arg){
  struct demod *demod = arg;
  adjust_item(demod,0);
}
void toggle_lock(void *arg){
  struct demod *demod = arg;
  switch(demod->tune.item){
  case 0:
  case 1:
    Frequency_lock = !Frequency_lock; // Toggle frequency tuning lock
    break;
  case 2:
    demod->tune.lock = !demod->tune.lock;
  }
}



#if 0 // highly incomplete, won't compile yet, will probably go away
double scrape_number(WINDOW *win, int y, int x, double **increment){
  char c;
  int i = 0;
  char buf[100];
  int max_y = 0,max_x = 0;
  int right_end = -1;
  int left_end = -1;
  getmaxyx(win,max_y,max_x);

  // Look for right end
  int multiplier = -1;
  int i;
  for(i=x; i < max_x; i++){
    c = mvwinch(win,y,x) & A_CHARTEXT;
    if(c == ',')
      continue;
    else if(isdigit(c))
      multiplier++;
    else
      break;
  }
  right_end = i-1;

  if(c != '.'){
    // Not found to right, look left
    multiplier = 0;
    for(i=x; i >= 0; i--){
    c = mvwinch(win,y,x) & A_CHARTEXT;
    if(c == ',')
      continue;
    else if(isdigit(c))
      multiplier--;
    else
      break;
    }
  }
  left_end = i;
  if(c != '.'){
    // No decimal point found
  }


  // look left for first element of field
  while(x > 0){
    c = mvwinch(win,y,x-1) & A_CHARTEXT;    
    if(!isdigit(c) && c != ',' && c != '.' && c != '-' && c != '+')
      break;
    x--;
  }


  while(x < max_x && i < sizeof(buf)){
    // To end of line
    c = mvwinch(win,y,x++) & A_CHARTEXT;
    if(isdigit(c) || c == '.' || c == '-' || c == '+')
      buf[i++] = c;
  }
  buf[i] = '\0';
  if(strlen(buf) == 0){
    if(increment)
      *increment = NULL; 
    return NAN;
  }
  return strtod(buf,NULL);
}

void increment(void){
  int c;

  c = mvinch(mod_y,mod_x) & A_CHARTEXT;
  if(!isdigit(c))
    return;

  while(1){
    if(c != '9'){
      c++;
      return;
    }
    c = '0';
    for(x = mod_x - 1; x >= 0; x--){
      c = mvinch(mod_y,x) & A_CHARTEXT;
      if(isdigit(c))
	break;
      if(
    }
    

  }
}
void decrement(void){
}

#endif


// Thread to display receiver state, updated at 10Hz by default
// Uses the ancient ncurses text windowing library
// Also services keyboard, mouse and tuning knob, if present
// I had been running this at normal priority, but it can start new demodulators
// so it must also run at preferred priority
void *display(void *arg){
  pthread_setname("display");
  assert(arg != NULL);
  struct demod * const demod = arg;

#ifdef linux
  extern void *knob(void *);
  pthread_t pthread_knob;
  pthread_create(&pthread_knob,NULL,knob,demod);

  extern void *touch(void *);
  pthread_t pthread_touch;
  pthread_create(&pthread_touch,NULL,touch,demod);   // Disable for now
#endif

  atexit(display_cleanup);

#if FUNCTIONKEYS
  slk_init(3);
#endif
  // talk directly to the terminal
  Tty = fopen("/dev/tty","r+");
  Term = newterm(NULL,Tty,Tty);
  set_term(Term);
  keypad(stdscr,TRUE);
  timeout(Update_interval); // update interval when nothing is typed
  cbreak();
  noecho();

  // Set up display subwindows
  int row = 0;
  int col = 0;
  WINDOW * const tuning = newwin(8,35,row,col);    // Frequency information
  col += 35;
  WINDOW * const sig = newwin(8,25,row,col); // Signal information
  col += 25;
  WINDOW * const info = newwin(8,42,row,col);     // Band information
  row += 8;
  col = 0;
  WINDOW * const filtering = newwin(12,22,row,col);
  col += 22;
  WINDOW * const demodulator = newwin(12,25,row,col);
  col += 25;
  WINDOW * const options = newwin(12,12,row,col); // Demod options
  col += 12;
  WINDOW * const sdr = newwin(12,25,row,col); // SDR information
  col += 25;

  WINDOW * const modes = newwin(Nmodes+2,7,row,col);
  col += Nmodes+2;

  col = 0;
  row += 12;
  WINDOW * const network = newwin(9,78,row,col); // Network status information
  col = 0;
  row += 9;
  WINDOW * const debug = newwin(8,78,row,col); // Note: overlaps function keys
  scrollok(debug,1);

  // A message from our sponsor...
  wprintw(debug,"KA9Q SDR Receiver v1.0; Copyright 2017-2018 Phil Karn\n");
  wprintw(debug,"Compiled on %s at %s\n",__DATE__,__TIME__);

  struct sockcache input_data_source,input_data_dest;
  memset(&input_data_source,0,sizeof(input_data_source));
  memset(&input_data_dest,0,sizeof(input_data_dest));
  
  struct sockcache output_data_source,output_data_dest;
  memset(&output_data_source,0,sizeof(output_data_source));
  memset(&output_data_dest,0,sizeof(output_data_dest));
  
  struct sockcache input_metadata_source,input_metadata_dest;
  memset(&input_metadata_source,0,sizeof(input_metadata_source));
  memset(&input_metadata_dest,0,sizeof(input_metadata_dest));  

  struct sockcache output_metadata_source,output_metadata_dest;
  memset(&output_metadata_source,0,sizeof(output_metadata_source));
  memset(&output_metadata_dest,0,sizeof(output_metadata_dest));

  mmask_t mask = ALL_MOUSE_EVENTS;
  mousemask(mask,NULL);
  MEVENT mouse_event;

  for(;;){
    // update display indefinitely, handle user commands

    // Tuning control window - these can be adjusted by the user
    // using the keyboard or tuning knob, so be careful with formatting
    wmove(tuning,0,0);
    int row = 1;
    int col = 1;
    if(Frequency_lock)
      wattron(tuning,A_UNDERLINE); // Underscore means the frequency is locked
    mvwprintw(tuning,row,col,"%'28.3f Hz",get_freq(demod)); // RF carrier frequency
    mvwaddstr(tuning,row,col,"Carrier");
    row++;

    // Center of passband
    mvwprintw(tuning,row,col,"%'28.3f Hz",get_freq(demod) + (demod->filter.high + demod->filter.low)/2);
    mvwaddstr(tuning,row++,col,"Center");

    wattroff(tuning,A_UNDERLINE);
    if(demod->tune.lock)
      wattron(tuning,A_UNDERLINE);    

    // second LO frequency is negative of IF, i.e., a signal at +48 kHz
    // needs a second LO frequency of -48 kHz to bring it to zero
    mvwprintw(tuning,row,col,"%'28.3f Hz",get_first_LO(demod));
    mvwaddstr(tuning,row++,col,"First LO");
    wattroff(tuning,A_UNDERLINE);

    mvwprintw(tuning,row,col,"%'28.3f Hz",-get_second_LO(demod));
    mvwaddstr(tuning,row++,col,"IF");

    // Doppler info displayed only if active
    double dopp = get_doppler(demod);
    if(dopp != 0){
      mvwprintw(tuning,row,col,"%'28.3f Hz",dopp);
      mvwaddstr(tuning,row++,col,"Doppler");
      mvwprintw(tuning,row,col,"%'28.3f Hz/s",get_doppler_rate(demod));
      mvwaddstr(tuning,row++,col,"Dop rate");
    }
    wmove(tuning,row,0);
    wclrtobot(tuning);

    box(tuning,0,0);
    mvwaddstr(tuning,0,15,"Tuning");


    // Display ham band emission data, if available
    // Lines are variable length, so clear window before starting
    wclrtobot(info);  // Output 
    row = 1;
    if(demod->doppler_command)
      mvwprintw(info,row++,1,"Doppler: %s",demod->doppler_command);

    struct bandplan const *bp_low,*bp_high;
    bp_low = lookup_frequency(get_freq(demod) + demod->filter.low);
    bp_high = lookup_frequency(get_freq(demod)+ demod->filter.high);
    // Make sure entire receiver passband is in the band
    if(bp_low != NULL && bp_high != NULL){
      struct bandplan r;

      // If the passband straddles a mode or class boundary, form
      // the intersection to give the more restrictive answers
      r.classes = bp_low->classes & bp_high->classes;
      r.modes = bp_low->modes & bp_high->modes;

      mvwprintw(info,row++,1,"Band: %s",bp_low->name);

      if(r.modes){
	mvwaddstr(info,row++,1,"Emissions: ");
	if(r.modes & VOICE)
	  waddstr(info,"Voice ");
	if(r.modes & IMAGE)
	  waddstr(info,"Image ");
	if(r.modes & DATA)
	  waddstr(info,"Data ");
	if(r.modes & CW)
	  waddstr(info,"CW "); // Last since it's permitted almost everywhere
      }
      if(r.classes){
	mvwaddstr(info,row++,1,"Privs: ");
	if(r.classes & EXTRA_CLASS)
	  waddstr(info,"Extra ");
	if(r.classes & ADVANCED_CLASS)
	  waddstr(info,"Adv ");
	if(r.classes & GENERAL_CLASS)
	  waddstr(info,"Gen ");
	if(r.classes & TECHNICIAN_CLASS)
	  waddstr(info,"Tech ");
	if(r.classes & NOVICE_CLASS)
	  waddstr(info,"Nov ");
      }
    }
    box(info,0,0);
    mvwaddstr(info,0,17,"Info");


    int const N = demod->filter.L + demod->filter.M - 1;
    // Filter window values
    row = 1;
    col = 1;
    mvwprintw(filtering,row,col,"%'+17.3f Hz",demod->filter.low);
    mvwaddstr(filtering,row++,col,"Low");
    mvwprintw(filtering,row,col,"%'+17.3f Hz",demod->filter.high);
    mvwaddstr(filtering,row++,col,"High");    
    mvwprintw(filtering,row,col,"%'+17.3f Hz",get_shift(demod));
    mvwaddstr(filtering,row++,col,"Shift");
    mvwprintw(filtering,row,col,"%'17.3f",demod->filter.kaiser_beta);
    mvwaddstr(filtering,row++,col,"Beta");    
    mvwprintw(filtering,row,col,"%'17d",demod->filter.L);
    mvwaddstr(filtering,row++,col,"Blocksize");
    mvwprintw(filtering,row,col,"%'17d",demod->filter.M);
    mvwaddstr(filtering,row++,col,"FIR");
    mvwprintw(filtering,row,col,"%'17.3f Hz",(float)demod->input.samprate / N);
    mvwaddstr(filtering,row++,col,"Freq bin");
    mvwprintw(filtering,row,col,"%'17.3f ms",1000.0*(N - (demod->filter.M - 1)/2)/demod->input.samprate); // Is this correct?
    mvwaddstr(filtering,row++,col,"Delay");
    mvwprintw(filtering,row,col,"%17d",demod->filter.interpolate);
    mvwaddstr(filtering,row++,col,"Interpolate");
    mvwprintw(filtering,row,col,"%17d",demod->filter.decimate);
    mvwaddstr(filtering,row++,col,"Decimate");

    box(filtering,0,0);
    mvwaddstr(filtering,0,6,"Filtering");


    // Signal data window
    float bw = 0;
    if(demod->filter.out != NULL)
      bw = demod->input.samprate * demod->filter.out->noise_gain;
    float sn0 = demod->sig.bb_power / demod->sig.n0 - bw;
    sn0 = max(sn0,0.0f); // Can go negative due to inconsistent smoothed values; clip it at zero

    row = 1;
    col = 1;
    mvwprintw(sig,row,col,"%15.1f dB",power2dB(demod->sig.if_power));
    mvwaddstr(sig,row++,col,"IF");
    mvwprintw(sig,row,col,"%15.1f dB",power2dB(demod->sig.bb_power));
    mvwaddstr(sig,row++,col,"Baseband");
    mvwprintw(sig,row,col,"%15.1f dB/Hz",power2dB(demod->sig.n0));
    mvwaddstr(sig,row++,col,"N0");
    mvwprintw(sig,row,col,"%15.1f dBHz",10*log10f(sn0));
    mvwaddstr(sig,row++,col,"S/N0");
    mvwprintw(sig,row,col,"%15.1f dBHz",10*log10f(bw));
    mvwaddstr(sig,row++,col,"NBW");
    mvwprintw(sig,row,col,"%15.1f dB",10*log10f(sn0/bw));
    mvwaddstr(sig,row++,col,"SNR");
    box(sig,0,0);
    mvwaddstr(sig,0,9,"Signal");


    // Demodulator info
    wmove(demodulator,0,0);
    wclrtobot(demodulator);    
    row = 1;
    int rcol = 9;
    int lcol = 1;
    // Display only if used by current mode
    switch(demod->demod_type){
    case AM_DEMOD:
      mvwprintw(demodulator,row,rcol,"%11.1f dB",voltage2dB(demod->agc.gain));
      mvwaddstr(demodulator,row++,lcol,"AF Gain");
      break;
    case FM_DEMOD:
      mvwprintw(demodulator,row,rcol,"%11.1f dB",power2dB(demod->sig.snr));
      mvwaddstr(demodulator,row++,lcol,"Input SNR");
      mvwprintw(demodulator,row,rcol,"%'+11.3f Hz",demod->sig.foffset);
      mvwaddstr(demodulator,row++,lcol,"Offset");
      mvwprintw(demodulator,row,rcol,"%11.1f Hz",demod->sig.pdeviation);
      mvwaddstr(demodulator,row++,lcol,"Deviation");
      mvwprintw(demodulator,row,rcol,"%11.1f Hz",demod->sig.plfreq);
      mvwaddstr(demodulator,row++,lcol,"PL Tone");
      break;
    case LINEAR_DEMOD:
      mvwprintw(demodulator,row,rcol,"%11.1f dB",voltage2dB(demod->agc.gain));
      mvwaddstr(demodulator,row++,lcol,"AF Gain");
      if(demod->opt.pll){
	mvwprintw(demodulator,row,rcol,"%11.1f dB",power2dB(demod->sig.snr));
	mvwaddstr(demodulator,row++,lcol,"PLL SNR");
	mvwprintw(demodulator,row,rcol,"%'+11.3f Hz",demod->sig.foffset);
	mvwaddstr(demodulator,row++,lcol,"Offset");
	mvwprintw(demodulator,row,rcol,"%+11.1f deg",demod->sig.cphase*DEGPRA);
	mvwaddstr(demodulator,row++,lcol,"PLL Phase");
	mvwprintw(demodulator,row,rcol,"%11s",demod->sig.pll_lock ? "Yes" : "No");
	mvwaddstr(demodulator,row++,lcol,"PLL Lock");
      }
      break;
    }
    box(demodulator,0,0);
    mvwprintw(demodulator,0,5,"%s demodulator",Demodtab[demod->demod_type].name);

    // SDR hardware status: sample rate, tcxo offset, I/Q offset and imbalance, gain settings
    row = 1;
    col = 1;
    mvwprintw(sdr,row,col,"%'18d Hz",demod->sdr.status.samprate); // Nominal
    mvwaddstr(sdr,row++,col,"Samprate");
    mvwprintw(sdr,row,col,"%'18.1f dBFS",
	      power2dB(demod->sig.if_power) + demod->sdr.status.lna_gain + demod->sdr.status.if_gain + demod->sdr.status.mixer_gain);
    mvwprintw(sdr,row++,col,"A/D Level");
    mvwprintw(sdr,row,col,"%18u dB",demod->sdr.status.lna_gain);   // SDR dependent
    mvwaddstr(sdr,row++,col,"LNA gain");
    mvwprintw(sdr,row,col,"%18u dB",demod->sdr.status.mixer_gain); // SDR dependent
    mvwaddstr(sdr,row++,col,"Mix gain");
    mvwprintw(sdr,row,col,"%18u dB",demod->sdr.status.if_gain); // SDR dependent    
    mvwaddstr(sdr,row++,col,"IF gain");
    mvwprintw(sdr,row,col,"%'21.6f",demod->sdr.DC_i);
    mvwaddstr(sdr,row++,col,"DC-i offs");
    mvwprintw(sdr,row,col,"%'21.6f",demod->sdr.DC_q);
    mvwaddstr(sdr,row++,col,"DC-q offs");
    mvwprintw(sdr,row,col,"%'18.1f deg",DEGPRA * asin(demod->sdr.sinphi));
    mvwaddstr(sdr,row++,col,"Phase offset");
    mvwprintw(sdr,row,col,"%'18.1f dB",voltage2dB(demod->sdr.imbalance));
    mvwaddstr(sdr,row++,col,"I/Q imbal");
    mvwprintw(sdr,row,col,"%'21lg",demod->sdr.calibration);
    mvwaddstr(sdr,row++,col,"TCXO cal");

    box(sdr,0,0);
    mvwaddstr(sdr,0,6,"SDR Hardware");


    // Demodulator options, can be set with mouse
    row = 1;
    col = 1;
    wclrtobot(options);    
    if(demod->demod_type == FM_DEMOD){ // FM from status.h
      if(demod->opt.flat)
	wattron(options,A_UNDERLINE);
      mvwprintw(options,row++,col,"FLAT");
      wattroff(options,A_UNDERLINE);
    } else if(demod->demod_type == LINEAR_DEMOD){
      if(demod->filter.isb)
	wattron(options,A_UNDERLINE);
      mvwprintw(options,row++,col,"ISB");
      wattroff(options,A_UNDERLINE);
      if(demod->opt.pll)
	wattron(options,A_UNDERLINE);      
      mvwprintw(options,row++,col,"PLL");
      wattroff(options,A_UNDERLINE);

      if(demod->opt.square)
	wattron(options,A_UNDERLINE);            
      mvwprintw(options,row++,col,"Square");
      wattroff(options,A_UNDERLINE);

      if(demod->output.channels == 1)
	wattron(options,A_UNDERLINE);
      mvwprintw(options,row++,col,"Mono");    
      wattroff(options,A_UNDERLINE);
      
      if(demod->output.channels == 2)
	wattron(options,A_UNDERLINE);
      mvwprintw(options,row++,col,"Stereo");    
      wattroff(options,A_UNDERLINE);
    }
    box(options,0,0);
    mvwaddstr(options,0,2,"Options");


    // Display list of modes defined in /usr/local/share/ka9q-radio/modes.txt
    // They're now presets, so they're no longer underlined
    // Can be selected with mouse
    row = 1; col = 1;
    for(int i=0;i<Nmodes;i++)
      mvwaddstr(modes,row++,col,Modes[i].name);

    box(modes,0,0);
    mvwaddstr(modes,0,1,"Modes");


    row = 1;
    col = 1;
    wmove(network,0,0);
    wclrtobot(network);
    update_sockcache(&input_metadata_source,(struct sockaddr *)&demod->input.metadata_source_address);
    update_sockcache(&input_metadata_dest,(struct sockaddr *)&demod->input.metadata_dest_address);
    mvwprintw(network,row++,col,"Input metadata: %s:%s -> %s:%s; pkts %'llu",
	      input_metadata_source.host,input_metadata_source.port,
	      input_metadata_dest.host,input_metadata_dest.port,
	      demod->input.metadata_packets);
    
    update_sockcache(&input_data_source,(struct sockaddr *)&demod->input.data_source_address);
    update_sockcache(&input_data_dest,(struct sockaddr *)&demod->input.data_dest_address);
    mvwprintw(network,row++,col,"Input data: %s:%s -> %s:%s; ssrc %0lx",
	      input_data_source.host,input_data_source.port,
	      input_data_dest.host,input_data_dest.port,
	      demod->input.rtp.ssrc);

    mvwprintw(network,row++,col,"  pkts %'llu samples %'llu",
	      demod->input.rtp.packets,demod->input.samples);

    if(demod->input.rtp.drops)
      wprintw(network," drops %'llu",demod->input.rtp.drops);
    if(demod->input.rtp.dupes)
      wprintw(network," dupes %'llu",demod->input.rtp.dupes);

    mvwprintw(network,row++,col,"Time: %s",lltime(demod->sdr.status.timestamp));
    update_sockcache(&output_metadata_source,(struct sockaddr *)&demod->output.metadata_source_address);
    update_sockcache(&output_metadata_dest,(struct sockaddr *)&demod->output.metadata_dest_address);
    mvwprintw(network,row++,col,"Output metadata: %s:%s -> %s:%s; pkts %'llu",
	      output_metadata_source.host,output_metadata_source.port,
	      output_metadata_dest.host,output_metadata_dest.port,
	      demod->output.metadata_packets);


    update_sockcache(&output_data_source,(struct sockaddr *)&demod->output.data_source_address);
    update_sockcache(&output_data_dest,(struct sockaddr *)&demod->output.data_dest_address);
    mvwprintw(network,row++,col,"Output data: %s:%s -> %s:%s; ssrc %8x; TTL %d%s",
	      output_data_source.host,output_data_source.port,
	      output_data_dest.host,output_data_dest.port,
	      demod->output.rtp.ssrc,Mcast_ttl,Mcast_ttl == 0 ? " (Local host only)":"");
    mvwprintw(network,row++,col,"PCM %'d Hz; pkts %'llu",demod->output.samprate,demod->output.rtp.packets);

    box(network,0,0);
    mvwaddstr(network,0,35,"I/O");

    touchwin(debug); // since we're not redrawing it every cycle

    // Highlight cursor for tuning step
    // A little messy because of the commas in the frequencies
    // They come from the ' option in the printf formats
    // tunestep is the log10 of the digit position (0 = units)
    int hcol;
    if(demod->tune.step >= 0){
      hcol = demod->tune.step + demod->tune.step/3;
      hcol = -hcol;
    } else {
      hcol = -demod->tune.step;
      hcol = 1 + hcol + (hcol-1)/3; // 1 for the decimal point, and extras if there were commas in more than 3 places
    }
    switch(demod->tune.item){
    case 0:
    case 1:
    case 2:
    case 3:
      mod_y = demod->tune.item + 1;
      mod_x = 24 + hcol; // units in column 24
      mvwchgat(tuning,mod_y,mod_x,1,A_STANDOUT,0,NULL);
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      mod_y = demod->tune.item - 3;
      mod_x = 13 + hcol; // units in column 13
      mvwchgat(filtering,mod_y,mod_x,1,A_STANDOUT,0,NULL);
      break;
    default:
      ;
      break;
    }
    wnoutrefresh(tuning);
    wnoutrefresh(debug);
    wnoutrefresh(info);
    wnoutrefresh(filtering);
    wnoutrefresh(sig);
    wnoutrefresh(demodulator);
    wnoutrefresh(sdr);
    wnoutrefresh(options);
    wnoutrefresh(modes);
    wnoutrefresh(network);
    doupdate();      // Update the screen right before we pause
    
    // Scan and process keyboard commands
    int c = getch(); // read keyboard with timeout; controls refresh rate

    switch(c){
    case KEY_MOUSE: // Mouse event
      getmouse(&mouse_event);
      break;
    case ERR:   // no key; timed out. Do nothing.
      break;
    case 'q':   // Exit entire radio program. Should this be removed? ^C also works.
      goto done;
    case 'h':
    case '?':
      popup("help.txt");
      break;
    case 'w': // Save radio state to file in ~/.radiostate
      {
	char str[160];
	getentry("Save state file: ",str,sizeof(str));
	if(strlen(str) > 0)
	  savestate(demod,str);
      }
      break;
    case 'I': // Change multicast address for input metadata
      {
	char str[160];
	getentry("IQ input IP dest address: ",str,sizeof(str));
	if(strlen(str) <= 0)
	  break;

	int const i = setup_mcast(str,(struct sockaddr *)&demod->input.metadata_dest_address,0,0,0);
	if(i == -1){
	  beep();
	  break;
	}
	// demod->input.data_fd is not protected by a mutex, so swap it carefully
	// Mutex protection would be difficult because input thread is usually
	// blocked on the socket, and if there's no I/Q input we'd hang
	int const j = demod->input.data_fd;
	demod->input.data_fd = i;
	if(j != -1)
	  close(j); // This should cause the input thread to see an error
	strlcpy(demod->input.dest_address_text,str,sizeof(demod->input.dest_address_text));
	// Clear RTP receiver state
	memset(&demod->input.rtp,0,sizeof(demod->input.rtp));
      }
      break;
    case 'l': // Toggle RF or first LO lock; affects how adjustments to LO and IF behave
      toggle_lock(demod);
      break;
    case KEY_NPAGE: // Page Down/tab key
    case '\t':      // go to next tuning item
      demod->tune.item = (demod->tune.item + 1) % 8;
      break;
    case KEY_BTAB:  // Page Up/Backtab, i.e., shifted tab:
    case KEY_PPAGE: // go to previous tuning item
      demod->tune.item = (8 + demod->tune.item - 1) % 8;
      break;
    case KEY_HOME: // Go back to item 0
      demod->tune.item = 0;
      demod->tune.step = 0;
      break;
    case KEY_BACKSPACE: // Cursor left: increase tuning step 10x
    case KEY_LEFT:
      if(demod->tune.step >= 9){
	beep();
	break;
      }
      demod->tune.step++;
      break;
    case KEY_RIGHT:     // Cursor right: decrease tuning step /10
      if(demod->tune.step <= -3){
	beep();
	break;
      }
      demod->tune.step--;
      break;
    case KEY_UP:        // Increase whatever digit we're tuning
      adjust_up(demod);
      break;
    case KEY_DOWN:      // Decrease whatever we're tuning
      adjust_down(demod);
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
	if(ptr == str)
	  break; // Nothing entered
	
	demod->filter.L = i;
	demod->filter.M = demod->filter.L + 1;
	// FIX THIS - need to restart demod thread
      }
      break;
    case 'm': // Manually set mode presets
      {
	char str[1024];
	snprintf(str,sizeof(str),"Enter mode [ ");
	for(int i=0;i < Nmodes;i++){
	  strlcat(str,Modes[i].name,sizeof(str) - strlen(str) - 1);
	  strlcat(str," ",sizeof(str) - strlen(str) - 1);
	}
	strlcat(str,"]: ",sizeof(str) - strlen(str) - 1);
	getentry(str,str,sizeof(str));
	if(strlen(str) <= 0)
	  break;
	preset_mode(demod,str);
      }
      break;
    case 'f':   // Tune to new radio frequency
      {
	char str[160];
	getentry("Enter carrier frequency: ",str,sizeof(str));
	double const f = parse_frequency(str); // Handles funky forms like 147m435
	if(f <= 0)
	  break; // Invalid

	// If frequency would be out of range, guess kHz or MHz
	if(f >= 0.1 && f < 100)
	  set_freq(demod,f*1e6,NAN); // 0.1 - 99.999 Only MHz can be valid
	else if(f < 500)         // 100-499.999 could be kHz or MHz, assume MHz
	  set_freq(demod,f*1e6,NAN);
	else if(f < 2000)        // 500-1999.999 could be kHz or MHz, assume kHz
	  set_freq(demod,f*1e3,NAN);
	else if(f < 100000)      // 2000-99999.999 can only be kHz
	  set_freq(demod,f*1e3,NAN);
	else                     // accept directly
	  set_freq(demod,f,NAN);
      }
      break;
    case 'i':    // Recenter IF to +/- samprate/4
      set_freq(demod,get_freq(demod),demod->input.samprate/4.);
      break;
    case 'u':    // Set display update rate in milliseconds (minimum 50, i.e, 20 Hz)
      {
	char str[160],*ptr;
	getentry("Enter update interval, ms [<=0 means no auto update]: ",str,sizeof(str));
	int const u = strtol(str,&ptr,0);
	if(ptr == str)
	  break; // Nothing entered
	
	if(u > 50){
	  Update_interval = u;
	  timeout(Update_interval);
	} else if(u <= 0){
	  Update_interval = -1; // No automatic update
	  timeout(Update_interval);
	} else
	  beep();
      }
      break;
    case 'k': // Kaiser window beta parameter
      {
	char str[160],*ptr;
	getentry("Enter Kaiser window beta: ",str,sizeof(str));
	double const b = strtod(str,&ptr);
	if(ptr == str)
	  break; // nothing entered
	if(b < 0 || b >= 100){
	  beep();
	  break; // beyond limits
	}
	if(b != demod->filter.kaiser_beta){
	  demod->filter.kaiser_beta = b;
	  set_filter(demod->filter.out,demod->filter.low/demod->input.samprate,demod->filter.high/demod->input.samprate,demod->filter.kaiser_beta);
	}
      }
      break;
    case 'o': // Set/clear option flags, most apply only to linear detector
      {
	char str[160];
	getentry("Enter option [isb pll cal flat square stereo mono], '!' prefix disables: ",str,sizeof(str));
	if(strcasecmp(str,"mono") == 0){
	  demod->output.channels = 1;
	} else if(strcasecmp(str,"!mono") == 0){
	  demod->output.channels = 2;
	} else if(strcasecmp(str,"stereo") == 0){
	  demod->output.channels = 2;
	} else if(strcasecmp(str,"isb") == 0){
	  demod->filter.isb = 1;
	} else if(strcasecmp(str,"!isb") == 0){
	  demod->filter.isb = 0;
	} else if(strcasecmp(str,"pll") == 0){
	  demod->opt.pll = 1;
	} else if(strcasecmp(str,"!pll") == 0){
	  demod->opt.pll = demod->opt.square = 0;
	} else if(strcasecmp(str,"square") == 0){
	  demod->opt.pll = demod->opt.square = 1;
	} else if(strcasecmp(str,"!square") == 0){	  
	  demod->opt.square = 0;
	} else if(strcasecmp(str,"flat") == 0){
	  demod->opt.flat = 1;
	} else if(strcasecmp(str,"!flat") == 0){
	  demod->opt.flat = 0;
	}
      }
      break;
    default:
      beep();
      break;
    }
    // Process mouse events
    // Need to handle the wheel as equivalent to up/down arrows
    int mx,my;
    mx = mouse_event.x;
    my = mouse_event.y;
    mouse_event.y = mouse_event.x = mouse_event.z = 0;
    if(mx != 0 && my != 0){
#if 0
      wprintw(debug," (%d %d)",mx,my);
#endif
      if(wmouse_trafo(tuning,&my,&mx,false)){
	// Tuning window
	demod->tune.item = my-1;
	demod->tune.step = 24-mx;
	if(demod->tune.step < 0)
	  demod->tune.step++;
	if(demod->tune.step > 3)
	  demod->tune.step--;
	if(demod->tune.step > 6)
	  demod->tune.step--;
	if(demod->tune.step > 9)	
	  demod->tune.step--;
	// Clamp to range
	if(demod->tune.step < -3)
	  demod->tune.step = -3;
	if(demod->tune.step > 9)
	  demod->tune.step = 9;

      } else if(wmouse_trafo(filtering,&my,&mx,false)){
	// Filter window
	demod->tune.item = my + 3;
	demod->tune.step = 13-mx;
	if(demod->tune.step < 0)
	  demod->tune.step++;
	if(demod->tune.step > 3)
	  demod->tune.step--;
	if(demod->tune.step > 6)
	  demod->tune.step--;
	if(demod->tune.step > 9)	
	  demod->tune.step--;
	// Clamp to range
	if(demod->tune.step < -3)
	  demod->tune.step = -3;
	if(demod->tune.step > 5)
	  demod->tune.step = 5;
      } else if(wmouse_trafo(modes,&my,&mx,false)){
	// In the modes window?
	my--;
	if(my >= 0 && my < Nmodes){
	  preset_mode(demod,Modes[my].name);
	}
      } else if(wmouse_trafo(options,&my,&mx,false)){
	// In the options window
	if(demod->demod_type == FM_DEMOD){
	  switch(my){
	  case 1:
	    demod->opt.flat = !demod->opt.flat;
	    break;
	  }
	} else if(demod->demod_type == LINEAR_DEMOD){
	  switch(my){
	  case 1:
	    demod->filter.isb = !demod->filter.isb;
	    break;
	  case 2:
	    demod->opt.pll = !demod->opt.pll;
	    break;
	  case 3:
	    demod->opt.square = !demod->opt.square;
	    if(demod->opt.square)
	      demod->opt.pll = 1;
	    break;
	  case 4:
	    demod->output.channels = 1;
	    break;
	  case 5:
	    demod->output.channels = 2;
	    break;
	  }
	}
      }
    }
  }
 done:;
  endwin();
  set_term(NULL);
  if(Term != NULL)
    delscreen(Term);
  //  if(Tty != NULL)
  //    fclose(Tty);
  
  // Dump receiver state to default
  savestate(demod,"default");
  exit(0);
}


// character size 16 pix high x 9 wide??
void touchitem(void *arg,int x,int y,int ev){
  touch_x = x /8;
  touch_y = y / 16;
}

// Parse a frequency entry in the form
// 12345 (12345 Hz)
// 12k345 (12.345 kHz)
// 12m345 (12.345 MHz)
// 12g345 (12.345 GHz)
// If no g/m/k and number is too small, make a heuristic guess
// NB! This assumes radio covers 100 kHz - 2 GHz; should make more general
double const parse_frequency(const char *s){
  char * const ss = alloca(strlen(s));

  int i;
  for(i=0;i<strlen(s);i++)
    ss[i] = tolower(s[i]);

  ss[i] = '\0';
  
  // k, m or g in place of decimal point indicates scaling by 1k, 1M or 1G
  char *sp;
  double mult;
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

  char *endptr = NULL;
  double f = strtod(ss,&endptr);
  if(endptr == ss || f == 0)
    return 0; // Empty entry, or nothing decipherable
  
  if(mult != 1 || f >= 1e5) // If multiplier given, or frequency >= 100 kHz (lower limit), return as-is
    return f * mult;
    
  // If frequency would be out of range, guess kHz or MHz
  if(f < 100)
    f *= 1e6;              // 0.1 - 99.999 Only MHz can be valid
  else if(f < 500)         // Could be kHz or MHz, arbitrarily assume MHz
    f *= 1e6;
  else if(f < 2000)        // Could be kHz or MHz, arbitarily assume kHz
    f *= 1e3;
  else if(f < 100000)      // Can only be kHz
    f *= 1e3;

  return f;
}
