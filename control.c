// $Id: control.c,v 1.38 2018/12/20 08:25:15 karn Exp karn $
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
#include <sys/select.h>
#include <ncurses.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netdb.h>
#include <locale.h>

#include "misc.h"
#include "dsp.h"
#include "radio.h"
#include "filter.h"
#include "multicast.h"
#include "bandplan.h"
#include "status.h"

int DAC_samprate = 48000;

// Touch screen position (Raspberry Pi display only - experimental)
int touch_x,touch_y;

// Screen location of field modification cursor
int mod_x,mod_y;

int Mcast_ttl = 1;

char Libdir[] = "/usr/local/share/ka9q-radio";
char Statepath[PATH_MAX];
char Locale[256] = "en_US.UTF-8";

struct sockaddr_storage SDR_status_address;

int SDR_status_fd = -1;
int Status_fd = -1;
int Ctl_fd = -1;

int Verbose,Dump;

// Dummy stubs to satisfy Demodtab in modes.c
void *demod_am(void *f){return NULL;}
void *demod_fm(void *f){return NULL;}
void *demod_linear(void *f){return NULL;}

void decode_radio_status(struct demod *demod,unsigned char *buffer,int length);


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
  timeout(0);
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
  timeout(0);
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
  
  tunestep = pow(10., (double)demod->tune.step);

  if(!direction)
    tunestep = - tunestep;

  switch(demod->tune.item){
  case 0: // Carrier frequency
  case 1: // Center frequency - treat the same
    if(!Frequency_lock) // Ignore if locked
      demod->tune.freq += tunestep;
    break;
  case 2: // First LO
    // this should ideally be sent directly to the front end, but that's work
    if(demod->tune.lock) // Tuner is locked, don't change it
      break;
    demod->sdr.status.frequency += tunestep;
    break;
  case 3: // IF
    demod->second_LO.freq -= tunestep;
    break;
  case 4: // Filter low edge
    demod->filter.low += tunestep;
    break;
  case 5: // Filter high edge
    demod->filter.high += tunestep;
    break;
  case 6: // Post-detection audio frequency shift
    demod->tune.shift += tunestep;
    demod->tune.freq += tunestep;
    break;
  case 7: // Kaiser window beta parameter for filter
    demod->filter.kaiser_beta += tunestep;
    if(demod->filter.kaiser_beta < 0)
      demod->filter.kaiser_beta = 0;
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


struct demod Demod;

float Noise_bandwidth = NAN;

void decode_sdr_status(struct demod *demod,unsigned char *buffer,int length);


// Thread to display receiver state, updated at 10Hz by default
// Uses the ancient ncurses text windowing library
// Also services keyboard, mouse and tuning knob, if present
// I had been running this at normal priority, but it can start new demodulators
// so it must also run at preferred priority
int main(int argc,char *argv[]){
  int c;

  while((c = getopt(argc,argv,"vd")) != EOF){
    switch(c){
    case 'v':
      Verbose++;
      break;
    }
  }

  {
    // The display thread assumes en_US.UTF-8, or anything with a thousands grouping character
    // Otherwise the cursor movements will be wrong
    char const * const cp = getenv("LANG");
    if(cp != NULL){
      strlcpy(Locale,cp,sizeof(Locale));
    }
  }
  setlocale(LC_ALL,Locale); // Set either the hardwired default or the value of $LANG if it exists
  snprintf(Statepath,sizeof(Statepath),"%s/%s",getenv("HOME"),".radiostate");
  Statepath[sizeof(Statepath)-1] = '\0';
  if(readmodes("modes.txt") != 0){
    fprintf(stderr,"Can't read mode table\n");
    exit(1);
  }

  struct demod * const demod = &Demod;
  // Set all floating point fields to NAN so they won't be displayed unless set at least once
  demod->sdr.calibration = demod->sdr.status.frequency = demod->sdr.DC_i = demod->sdr.DC_q =
    demod->sdr.sinphi = demod->sdr.imbalance = demod->sdr.min_IF = demod->sdr.max_IF = demod->sdr.gain_factor = NAN;

  demod->tune.freq = demod->tune.shift = NAN;
  demod->second_LO.freq = NAN;
  demod->filter.low = demod->filter.high = demod->filter.kaiser_beta = demod->filter.noise_bandwidth = NAN;
  demod->agc.headroom = demod->agc.hangtime = demod->agc.recovery_rate = demod->agc.attack_rate = NAN;
  demod->sig.if_power = demod->sig.bb_power = demod->sig.n0 = demod->sig.snr = demod->sig.foffset = NAN;
  demod->sig.pdeviation = demod->sig.cphase = demod->sig.plfreq = demod->sig.lock_timer = NAN;
  demod->agc.gain = 1;
  

  Status_fd = setup_mcast(argv[optind],(struct sockaddr *)&demod->output.metadata_dest_address,0,Mcast_ttl,2);
  if(Status_fd == -1){
    fprintf(stderr,"Can't listen to %s\n",argv[optind]);
    exit(1);
  }
  Ctl_fd = setup_mcast(NULL,(struct sockaddr *)&demod->output.metadata_dest_address,1,Mcast_ttl,2);

  atexit(display_cleanup);

  WINDOW *tuning,*sig,*info,*filtering,*demodulator,*options,*sdr,*modes,*debug,*data,*status;

  // talk directly to the terminal
  Tty = fopen("/dev/tty","r+");
  Term = newterm(NULL,Tty,Tty);
  set_term(Term);
  keypad(stdscr,TRUE);
  timeout(0); // Don't block in getch()
  cbreak();
  noecho();
  // Set up display subwindows

  // First row
  int row = 0;
  int col = 0;
  tuning = newwin(9,35,row,col);    // Frequency information
  col += 35;
  sig = newwin(9,25,row,col); // Signal information
  col += 25;
  info = newwin(9,42,row,col);     // Band information
  col += 42;
  modes = newwin(Nmodes+2,7,row,col);
  col += 7;

  // Second row
  row += 9;
  col = 0;
  filtering = newwin(12,22,row,col);
  col += 22;
  demodulator = newwin(12,26,row,col);
  col += 26;
  options = newwin(12,12,row,col); // Demod options
  col += 12;
  sdr = newwin(12,30,row,col); // SDR information
  col += 30;
  
  // Third row
  col = 0;
  row += 12;
  data = newwin(5,109,row,col);
  col = 0;
  row += 5;
  status = newwin(5,109,row,col);
  col = 0;
  row += 5;
  debug = newwin(8,109,row,col); // Note: overlaps function keys
  scrollok(debug,1);
  
  // A message from our sponsor...
  wprintw(debug,"KA9Q SDR Receiver controller v1.0; Copyright 2018 Phil Karn\n");
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
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    fd_set fdset;
    FD_ZERO(&fdset);
    if(SDR_status_fd != -1)
      FD_SET(SDR_status_fd,&fdset);
    FD_SET(Status_fd,&fdset);
    int n = max(SDR_status_fd,Status_fd) + 1;
    n = select(n,&fdset,NULL,NULL,&timeout);

    if(FD_ISSET(Status_fd,&fdset)){
      unsigned char buffer[8192];
      memset(buffer,0,sizeof(buffer));
      socklen_t ssize = sizeof(demod->output.metadata_source_address);
      int length = recvfrom(Status_fd,buffer,sizeof(buffer),0,(struct sockaddr *)&demod->output.metadata_source_address,&ssize);
      if(length <= 0){
	usleep(100000);
	continue;
      }
      int cr = buffer[0]; // Command/response byte
      // Parse entries
      if(cr == 1)
	continue;     // Ignore commands

      struct demod ndemod;
      memcpy(&ndemod,demod,sizeof(ndemod));
      decode_radio_status(&ndemod,buffer+1,length-1);
      // Listen directly to the front end once we know who it is
      if(memcmp(&ndemod.input.metadata_dest_address,&demod->input.metadata_dest_address,sizeof(ndemod.input.metadata_dest_address)) != 0){
	if(SDR_status_fd > 0){
	  close(SDR_status_fd);
	  SDR_status_fd = -1;
	}
	SDR_status_fd = setup_mcast(NULL,(struct sockaddr *)&ndemod.input.metadata_dest_address,0,0,0);
      }
      memcpy(demod,&ndemod,sizeof(*demod));
    }
    if(SDR_status_fd != -1 && FD_ISSET(SDR_status_fd,&fdset)){
      unsigned char buffer[8192];
      memset(buffer,0,sizeof(buffer));
      int length = recv(SDR_status_fd,buffer,sizeof(buffer),0);
      if(length <= 0){
	usleep(100000);
	continue;
      }
      int cr = buffer[0]; // Command/response byte
      // Parse entries
      if(cr == 1)
	continue;     // Ignore commands

      decode_sdr_status(demod,buffer+1,length-1);
    }
    // update display indefinitely, handle user commands

    // Tuning control window - these can be adjusted by the user
    // using the keyboard or tuning knob, so be careful with formatting
    wmove(tuning,0,0);
    int row = 1;
    int col = 1;
    if(!isnan(demod->tune.freq)){
      if(Frequency_lock)
	wattron(tuning,A_UNDERLINE); // Underscore means the frequency is locked
      mvwprintw(tuning,row,col,"%'28.3f Hz",demod->tune.freq); // RF carrier frequency
      mvwaddstr(tuning,row,col,"Carrier");
      row++;
      // Center of passband
      if(!isnan(demod->filter.high) && !isnan(demod->filter.low)){
	mvwprintw(tuning,row,col,"%'28.3f Hz",demod->tune.freq + (demod->filter.high + demod->filter.low)/2);
	mvwaddstr(tuning,row++,col,"Center");
	wattroff(tuning,A_UNDERLINE);
	if(demod->tune.lock)
	  wattron(tuning,A_UNDERLINE);    
      }
    }      
    // second LO frequency is negative of IF, i.e., a signal at +48 kHz
    // needs a second LO frequency of -48 kHz to bring it to zero
    if(!isnan(demod->sdr.status.frequency)){
      mvwprintw(tuning,row,col,"%'28.3f Hz",demod->sdr.status.frequency);
      mvwaddstr(tuning,row++,col,"First LO");
      wattroff(tuning,A_UNDERLINE);
    }
    if(!isnan(demod->second_LO.freq)){
      mvwprintw(tuning,row,col,"%'28.3f Hz",-demod->second_LO.freq);
      mvwaddstr(tuning,row++,col,"IF");
    }

    // Doppler info displayed only if active
    double dopp = demod->doppler.freq;
    if(!isnan(dopp) && dopp != 0){
      mvwprintw(tuning,row,col,"%'28.3f Hz",dopp);
      mvwaddstr(tuning,row++,col,"Doppler");
      mvwprintw(tuning,row,col,"%'28.3f Hz/s",demod->doppler.rate);
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
    col = 1;
    mvwprintw(info,row++,col,"%s",lltime(demod->sdr.status.timestamp));
    mvwprintw(info,row++,col,"%s",demod->input.description);

    struct bandplan const *bp_low,*bp_high;
    bp_low = lookup_frequency(demod->tune.freq + demod->filter.low);
    bp_high = lookup_frequency(demod->tune.freq + demod->filter.high);
    // Make sure entire receiver passband is in the band
    if(bp_low != NULL && bp_high != NULL){
      struct bandplan r;

      // If the passband straddles a mode or class boundary, form
      // the intersection to give the more restrictive answers
      r.classes = bp_low->classes & bp_high->classes;
      r.modes = bp_low->modes & bp_high->modes;

      mvwprintw(info,row++,col,"Band: %s",bp_low->name);

      if(r.modes){
	mvwaddstr(info,row++,col,"Emissions: ");
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
	mvwaddstr(info,row++,col,"Privs: ");
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
    if(!isnan(demod->filter.low)){
      mvwprintw(filtering,row,col,"%'+17.3f Hz",demod->filter.low);
      mvwaddstr(filtering,row++,col,"Low");
    }
    if(!isnan(demod->filter.high)){
      mvwprintw(filtering,row,col,"%'+17.3f Hz",demod->filter.high);
      mvwaddstr(filtering,row++,col,"High");    
    }
    if(!isnan(demod->tune.shift)){
      mvwprintw(filtering,row,col,"%'+17.3f Hz",demod->tune.shift);
      mvwaddstr(filtering,row++,col,"Shift");
    }
    if(!isnan(demod->filter.kaiser_beta)){
      mvwprintw(filtering,row,col,"%'17.3f",demod->filter.kaiser_beta);
      mvwaddstr(filtering,row++,col,"Beta");    
    }
    if(demod->filter.L > 0){
      mvwprintw(filtering,row,col,"%'17d",demod->filter.L);
      mvwaddstr(filtering,row++,col,"Blocksize");
    }
    if(demod->filter.M > 0){
      mvwprintw(filtering,row,col,"%'17d",demod->filter.M);
      mvwaddstr(filtering,row++,col,"FIR");
    }
    if(demod->input.samprate > 0 && N > 0){
      mvwprintw(filtering,row,col,"%'17.3f Hz",(float)demod->input.samprate / N);
      mvwaddstr(filtering,row++,col,"Freq bin");
      mvwprintw(filtering,row,col,"%'17.3f ms",1000.0*(N - (demod->filter.M - 1)/2)/demod->input.samprate); // Is this correct?
      mvwaddstr(filtering,row++,col,"Delay");
    }
#if 0
    mvwprintw(filtering,row,col,"%17d",demod->filter.interpolate);
    mvwaddstr(filtering,row++,col,"Interpolate");
#endif
    if(demod->filter.decimate > 1){
      mvwprintw(filtering,row,col,"%17d",demod->filter.decimate);
      mvwaddstr(filtering,row++,col,"Decimate");
    }
    box(filtering,0,0);
    mvwaddstr(filtering,0,6,"Filtering");


    // Signal data window
    float sig_power = dB2power(demod->sig.bb_power) - Noise_bandwidth * dB2power(demod->sig.n0);
    if(sig_power < 0)
      sig_power = 0;
    float bw = power2dB(Noise_bandwidth);

    float sn0 = power2dB(sig_power) - demod->sig.n0;

    row = 1;
    col = 1;
    if(!isnan(demod->sig.if_power)){
      mvwprintw(sig,row,col,"%15.1f dB",demod->sig.if_power);
      mvwaddstr(sig,row++,col,"IF");
    }
    if(!isnan(demod->sig.bb_power)){
      mvwprintw(sig,row,col,"%15.1f dB",demod->sig.bb_power);
      mvwaddstr(sig,row++,col,"Baseband");
    }
    if(!isnan(demod->sig.n0)){
      mvwprintw(sig,row,col,"%15.1f dB/Hz",demod->sig.n0);
      mvwaddstr(sig,row++,col,"N0");
    }
    if(!isnan(sn0)){
      mvwprintw(sig,row,col,"%15.1f dBHz",sn0);
      mvwaddstr(sig,row++,col,"S/N0");
    }
    if(!isnan(bw)){
      mvwprintw(sig,row,col,"%15.1f dBHz",bw);
      mvwaddstr(sig,row++,col,"NBW");
      if(!isnan(sn0)){
	mvwprintw(sig,row,col,"%15.1f dB",sn0 - bw);
	mvwaddstr(sig,row++,col,"SNR");
      }
    }
    if(!isnan(demod->output.level)){
      mvwprintw(sig,row,col,"%15.1lf dB",demod->output.level);
      mvwaddstr(sig,row++,col,"Output");
    }
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
    case FM_DEMOD:
      if(!isnan(demod->sig.snr)){
	mvwprintw(demodulator,row,rcol,"%11.1f dB",demod->sig.snr);
	mvwaddstr(demodulator,row++,lcol,"Input SNR");
      }
      if(!isnan(demod->sig.foffset)){
	mvwprintw(demodulator,row,rcol,"%'+11.3f Hz",demod->sig.foffset);
	mvwaddstr(demodulator,row++,lcol,"Offset");
      }
      if(!isnan(demod->sig.pdeviation)){
	mvwprintw(demodulator,row,rcol,"%11.1f Hz",demod->sig.pdeviation);
	mvwaddstr(demodulator,row++,lcol,"Deviation");
      }
      if(!isnan(demod->sig.plfreq)){
	mvwprintw(demodulator,row,rcol,"%11.1f Hz",demod->sig.plfreq);
	mvwaddstr(demodulator,row++,lcol,"PL Tone");
      }
      break;
    case LINEAR_DEMOD:
      if(!isnan(demod->agc.gain)){
	mvwprintw(demodulator,row,rcol,"%11.1f dB",demod->agc.gain);
	mvwaddstr(demodulator,row++,lcol,"Gain");
      }
      if(!isnan(demod->agc.recovery_rate)){
	mvwprintw(demodulator,row,rcol,"%11.1f dB/s",demod->agc.recovery_rate);
	mvwaddstr(demodulator,row++,lcol,"Recovery rate");
      }
      if(!isnan(demod->agc.hangtime)){
	mvwprintw(demodulator,row,rcol,"%11.1f s",demod->agc.hangtime);
	mvwaddstr(demodulator,row++,lcol,"Hang time");
      }
      if(!isnan(demod->agc.headroom)){
	mvwprintw(demodulator,row,rcol,"%11.1f dB",demod->agc.headroom);
	mvwaddstr(demodulator,row++,lcol,"Headroom");
      }
      if(demod->opt.pll){
	if(!isnan(demod->sig.snr)){
	  mvwprintw(demodulator,row,rcol,"%11.1f dB",demod->sig.snr);
	  mvwaddstr(demodulator,row++,lcol,"PLL SNR");
	}
	if(!isnan(demod->sig.foffset)){
	  mvwprintw(demodulator,row,rcol,"%'+11.3f Hz",demod->sig.foffset);
	  mvwaddstr(demodulator,row++,lcol,"Offset");
	}
	if(!isnan(demod->sig.cphase)){
	  mvwprintw(demodulator,row,rcol,"%+11.1f deg",demod->sig.cphase*DEGPRA);
	  mvwaddstr(demodulator,row++,lcol,"PLL Phase");
	}
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
    mvwprintw(sdr,row,col,"%'23d Hz",demod->sdr.status.samprate); // Nominal
    mvwaddstr(sdr,row++,col,"Samprate");
    mvwprintw(sdr,row,col,"%'23.1f dBFS",demod->sdr.ad_level);
    mvwprintw(sdr,row++,col,"A/D Level");
    mvwprintw(sdr,row++,col,"Analog gain    %02d+%02d+%02d dB",demod->sdr.status.lna_gain,
	      demod->sdr.status.mixer_gain,
	      demod->sdr.status.if_gain);

    if(!isnan(demod->sdr.DC_i)){
      mvwprintw(sdr,row,col,"%'23.6f",demod->sdr.DC_i);
      mvwaddstr(sdr,row++,col,"DC-i offs");
    }
    if(!isnan(demod->sdr.DC_q)){
      mvwprintw(sdr,row,col,"%'23.6f",demod->sdr.DC_q);
      mvwaddstr(sdr,row++,col,"DC-q offs");
    }
    if(!isnan(demod->sdr.sinphi)){
      mvwprintw(sdr,row,col,"%'23.1f deg",DEGPRA * asin(demod->sdr.sinphi));
      mvwaddstr(sdr,row++,col,"Phase offset");
    }
    if(!isnan(demod->sdr.imbalance)){
      mvwprintw(sdr,row,col,"%'23.1f dB",demod->sdr.imbalance);
      mvwaddstr(sdr,row++,col,"I/Q imbal");
    }
    if(!isnan(demod->sdr.calibration)){
      mvwprintw(sdr,row,col,"%'23lg",demod->sdr.calibration);
      mvwaddstr(sdr,row++,col,"TCXO cal");
    }
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

      if(demod->opt.env)
	wattron(options,A_UNDERLINE);	
      mvwprintw(options,row++,col,"Envel");    
      wattroff(options,A_UNDERLINE);

      if(demod->opt.agc)
	wattron(options,A_UNDERLINE);
      mvwprintw(options,row++,col,"AGC");
      wattroff(options,A_UNDERLINE);      

    }
    box(options,0,0);
    mvwaddstr(options,0,2,"Options");


    // Display list of modes defined in /usr/local/share/ka9q-radio/modes.txt
    // These are now really presets that select a demodulator and parameters,
    // so they're no longer underlined
    // Can be selected with mouse
    row = 1; col = 1;
    for(int i=0;i<Nmodes;i++)
      mvwaddstr(modes,row++,col,Modes[i].name);
    box(modes,0,0);
    mvwaddstr(modes,0,1,"Modes");

    update_sockcache(&input_data_source,(struct sockaddr *)&demod->input.data_source_address);
    update_sockcache(&input_data_dest,(struct sockaddr *)&demod->input.data_dest_address);
    update_sockcache(&output_data_source,(struct sockaddr *)&demod->output.data_source_address);
    update_sockcache(&output_data_dest,(struct sockaddr *)&demod->output.data_dest_address);
    update_sockcache(&input_metadata_source,(struct sockaddr *)&demod->input.metadata_source_address);
    update_sockcache(&input_metadata_dest,(struct sockaddr *)&demod->input.metadata_dest_address);
    update_sockcache(&output_metadata_source,(struct sockaddr *)&demod->output.metadata_source_address);
    update_sockcache(&output_metadata_dest,(struct sockaddr *)&demod->output.metadata_dest_address);

    wmove(data,0,0);
    wclrtobot(data);
    row = 2;  col = 1;
    mvwprintw(data,row++,col,"in");
    mvwprintw(data,row,col,"out");
    col += 4;

    row = 1;
    mvwprintw(data,row++,col,"%12s","packets");
    mvwprintw(data,row++,col,"%'12llu",demod->input.rtp.packets);
    mvwprintw(data,row,col,"%'12llu",demod->output.rtp.packets);
    col += 13;

    row = 1;
    mvwprintw(data,row++,col,"%16s","samples");
    mvwprintw(data,row++,col,"%'16llu",demod->input.samples);
    mvwprintw(data,row,col,"%'16llu",demod->output.samples);
    col += 17;

    row = 1;
    mvwprintw(data,row++,col,"%6s","drops");
    mvwprintw(data,row,col,"%'6llu",demod->input.rtp.drops);
    col += 7;

    row = 1;
    mvwprintw(data,row++,col,"%6s","dupes");
    mvwprintw(data,row,col,"%'6llu",demod->input.rtp.dupes);
    col += 7;

    row = 1;
    mvwprintw(data,row++,col,"%8s","ssrc");
    mvwprintw(data,row++,col,"%8x",demod->input.rtp.ssrc);
    mvwprintw(data,row,col,"%8x",demod->output.rtp.ssrc);
    col += 9;

    row = 1;
    mvwprintw(data,row++,col,"socket");
    mvwprintw(data,row++,col,"%s:%s -> %s:%s",
	      input_data_source.host,input_data_source.port,
	      input_data_dest.host,input_data_dest.port);
    mvwprintw(data,row,col,"%s:%s -> %s:%s",
	      output_data_source.host,output_data_source.port,
	      output_data_dest.host,output_data_dest.port);
    
    box(data,0,0);
    mvwaddstr(data,0,6,"Data");

    wmove(status,0,0);
    wclrtobot(status);
    row = 2;  col = 1;
    mvwprintw(status,row++,col,"in");
    mvwprintw(status,row,col,"out");
    col += 4;

    row = 1;
    mvwprintw(status,row++,col,"%12s","packets");
    mvwprintw(status,row++,col,"%'12llu",demod->input.metadata_packets);
    mvwprintw(status,row,col,"%'12llu",demod->output.metadata_packets);
    col += 13;

    row = 1;
    mvwprintw(status,row++,col,"%12s","commands");
    mvwprintw(status,row++,col,"%'12llu",demod->input.commands);
    mvwprintw(status,row,col,"%'12llu",demod->output.commands);    
    col += 13;

    row = 1;
    mvwprintw(status,row++,col,"socket");
    mvwprintw(status,row++,col,"%s:%s -> %s:%s",
	      input_metadata_source.host,input_metadata_source.port,
	      input_metadata_dest.host,input_metadata_dest.port);
    mvwprintw(status,row,col,"%s:%s -> %s:%s",
	      output_metadata_source.host,output_metadata_source.port,
	      output_metadata_dest.host,output_metadata_dest.port);

    box(status,0,0);
    mvwaddstr(status,0,6,"Status");

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
    wnoutrefresh(data);
    wnoutrefresh(status);
    doupdate();      // Update the screen right before we pause
    
    // Scan and process keyboard commands
    int c = getch(); // read keyboard with timeout; controls refresh rate
    if(c == ERR)
      continue;   // no key; timed out. Do nothing.

    struct demod old_demod;
    memcpy(&old_demod,demod,sizeof(old_demod));

    switch(c){
    case KEY_MOUSE: // Mouse event
      getmouse(&mouse_event);
      break;
    case 'q':   // Exit entire radio program. Should this be removed? ^C also works.
      goto done;
    case 'h':
    case '?':
      popup("help.txt");
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
    case 'm': // Manually set modulation mode
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
	  demod->tune.freq = f*1e6; // 0.1 - 99.999 Only MHz can be valid
	else if(f < 500)         // 100-499.999 could be kHz or MHz, assume MHz
	  demod->tune.freq = f*1e6;
	else if(f < 2000)        // 500-1999.999 could be kHz or MHz, assume kHz
	  demod->tune.freq = f*1e3;
	else if(f < 100000)      // 2000-99999.999 can only be kHz
	  demod->tune.freq = f*1e3;
	else                     // accept directly
	  demod->tune.freq = f;
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
	demod->filter.kaiser_beta = b;
      }
      break;
    case 'o': // Set/clear option flags, most apply only to linear detector
      {
	char str[160];
	getentry("Enter option [isb pll flat square stereo mono agc], '!' prefix disables: ",str,sizeof(str));
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
	} else if(strcasecmp(str,"agc") == 0){
	  demod->opt.agc = 1;
	} else if(strcasecmp(str,"!agc") == 0){
	  demod->opt.agc = 0;
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
	if(my >= 0 && my < Nmodes)
	  preset_mode(demod,Modes[my].name);

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
	  case 6:
	    demod->opt.env = !demod->opt.env;
	    break;
	  case 7:
	    demod->opt.agc = !demod->opt.agc;
	    break;
	  }
	}
      }
    }
    // OK, what changed?
    // NB: NaNs always compare unequal, even with themselves. This will cause
    // encode_double() and encode_float() to be called when their fields are NAN
    // There are checks in encode_double() and encode_float() to suppress encoding
    // Is this the right thing to do?
    {
      unsigned char cmd_packet[8192],*bp;
      memset(cmd_packet,0,sizeof(cmd_packet));
      bp = cmd_packet;
      *bp++ = 1; // Command

      if(demod->demod_type != old_demod.demod_type)
	encode_int(&bp,DEMOD_TYPE,demod->demod_type);

      if(demod->tune.freq != old_demod.tune.freq)
	encode_double(&bp,RADIO_FREQUENCY,demod->tune.freq);

      if(demod->sdr.status.frequency != old_demod.sdr.status.frequency)
	encode_double(&bp,FIRST_LO_FREQUENCY,demod->sdr.status.frequency);

      if(demod->second_LO.freq != old_demod.second_LO.freq)
	encode_double(&bp,SECOND_LO_FREQUENCY,demod->second_LO.freq);

      if(demod->filter.high != old_demod.filter.high)
	encode_float(&bp,HIGH_EDGE,demod->filter.high);

      if(demod->filter.low != old_demod.filter.low)
	encode_float(&bp,LOW_EDGE,demod->filter.low);

      if(demod->tune.shift != old_demod.tune.shift)
	encode_float(&bp,SHIFT_FREQUENCY,demod->tune.shift);

      if(demod->filter.kaiser_beta != old_demod.filter.kaiser_beta)
	encode_float(&bp,KAISER_BETA,demod->filter.kaiser_beta);

      if(demod->output.channels != old_demod.output.channels)
	encode_int(&bp,OUTPUT_CHANNELS,demod->output.channels);

      if(demod->filter.isb != old_demod.filter.isb)
	encode_int(&bp,INDEPENDENT_SIDEBAND,demod->filter.isb);

      if(demod->opt.pll != old_demod.opt.pll)
	encode_int(&bp,PLL_ENABLE,demod->opt.pll);

      if(demod->opt.square != old_demod.opt.square)
	encode_int(&bp,PLL_SQUARE,demod->opt.square);	

      if(demod->opt.flat != old_demod.opt.flat)
	encode_int(&bp,FM_FLAT,demod->opt.flat);

      if(demod->agc.headroom != old_demod.agc.headroom)
	encode_float(&bp,HEADROOM,demod->agc.headroom);

      if(demod->agc.hangtime != old_demod.agc.hangtime)
	encode_float(&bp,AGC_HANGTIME,demod->agc.hangtime);
	
      if(demod->agc.attack_rate != old_demod.agc.attack_rate)
	encode_float(&bp,AGC_ATTACK_RATE,demod->agc.attack_rate);

      if(demod->agc.recovery_rate != old_demod.agc.recovery_rate)
	encode_float(&bp,AGC_RECOVERY_RATE,demod->agc.recovery_rate);

      if(demod->opt.env != old_demod.opt.env)
	encode_byte(&bp,ENVELOPE,demod->opt.env);

      if(demod->opt.agc != old_demod.opt.agc)
	encode_byte(&bp,AGC_ENABLE,demod->opt.agc);	

      demod->output.command_tag = random();
      encode_int(&bp,COMMAND_TAG,demod->output.command_tag);

      encode_eol(&bp);
      send(Ctl_fd, cmd_packet, bp - cmd_packet, 0);
    }
  }
 done:;
  endwin();
  set_term(NULL);
  if(Term != NULL)
    delscreen(Term);
  //  if(Tty != NULL)
  //    fclose(Tty);
  
  exit(0);
}


// character size 16 pix high x 9 wide??
void touchitem(void *arg,int x,int y,int ev){
  touch_x = x /8;
  touch_y = y / 16;
}




void decode_radio_status(struct demod *demod,unsigned char *buffer,int length){
  unsigned char *cp = buffer;

  while(cp - buffer < length){
    enum status_type type = *cp++; // increment cp to length field
    
    if(type == EOL)
      break; // End of list

    unsigned int optlen = *cp++;
    if(cp - buffer + optlen >= length)
      break; // Invalid length
    switch(type){
    case EOL: // Shouldn't get here
      goto done;
    case DESCRIPTION:
      decode_string(cp,optlen,&demod->input.description,sizeof(demod->input.description));
      break;
    case COMMANDS:
      demod->output.commands = decode_int(cp,optlen);
      break;
    case GPS_TIME:
      demod->sdr.status.timestamp = decode_int(cp,optlen);
      break;
    case INPUT_DATA_SOURCE_SOCKET:
      decode_socket(&demod->input.data_source_address,cp,optlen);
      break;
    case INPUT_DATA_DEST_SOCKET:
      decode_socket(&demod->input.data_dest_address,cp,optlen);
      break;
    case INPUT_METADATA_SOURCE_SOCKET:
      decode_socket(&demod->input.metadata_source_address,cp,optlen);
      break;
    case INPUT_METADATA_DEST_SOCKET:
      decode_socket(&demod->input.metadata_dest_address,cp,optlen);
      break;
    case INPUT_SSRC:
      demod->input.rtp.ssrc = decode_int(cp,optlen);
      break;
    case INPUT_SAMPRATE:
      demod->input.samprate = demod->sdr.status.samprate = decode_int(cp,optlen);
      break;
    case INPUT_DATA_PACKETS:
      demod->input.rtp.packets = decode_int(cp,optlen);
      break;
    case INPUT_METADATA_PACKETS:
      demod->input.metadata_packets = decode_int(cp,optlen);
      break;
    case INPUT_SAMPLES:
      demod->input.samples = decode_int(cp,optlen);
      break;
    case INPUT_DROPS:
      demod->input.rtp.drops = decode_int(cp,optlen);
      break;
    case INPUT_DUPES:
      demod->input.rtp.dupes = decode_int(cp,optlen);
      break;
    case OUTPUT_DATA_SOURCE_SOCKET:
      decode_socket(&demod->output.data_source_address,cp,optlen);
      break;
    case OUTPUT_DATA_DEST_SOCKET:
      decode_socket(&demod->output.data_dest_address,cp,optlen);
      break;
    case OUTPUT_SSRC:
      demod->output.rtp.ssrc = decode_int(cp,optlen);
      break;
    case OUTPUT_TTL:
      Mcast_ttl = decode_int(cp,optlen);
      break;
    case OUTPUT_SAMPRATE:
      demod->output.samprate = decode_int(cp,optlen);
      demod->filter.interpolate = 1; // Placeholder
      demod->filter.decimate = demod->input.samprate / demod->output.samprate;
      break;
    case OUTPUT_DATA_PACKETS:
      demod->output.rtp.packets = decode_int(cp,optlen);
      break;
    case OUTPUT_METADATA_PACKETS:
      demod->output.metadata_packets = decode_int(cp,optlen);
      break;
    case RADIO_FREQUENCY:
      demod->tune.freq = decode_double(cp,optlen);
      break;
    case FIRST_LO_FREQUENCY:
      demod->sdr.status.frequency = decode_double(cp,optlen);
      break;
    case SECOND_LO_FREQUENCY:
      demod->second_LO.freq = decode_double(cp,optlen);
      break;
    case SHIFT_FREQUENCY:
      demod->tune.shift = decode_double(cp,optlen);
      break;
    case DOPPLER_FREQUENCY:
      demod->doppler.freq = decode_double(cp,optlen);
      break;
    case DOPPLER_FREQUENCY_RATE:
      demod->doppler.rate = decode_double(cp,optlen);
      break;

      // Process SDR parameters in case we're looking at it directly
    case AD_LEVEL:
      demod->sdr.ad_level = decode_float(cp,optlen);
      break;
    case LNA_GAIN:
      demod->sdr.status.lna_gain = decode_int(cp,optlen);
      break;
    case MIXER_GAIN:
      demod->sdr.status.mixer_gain = decode_int(cp,optlen);
      break;
    case IF_GAIN:
      demod->sdr.status.if_gain = decode_int(cp,optlen);
      break;
    case DC_I_OFFSET:
      demod->sdr.DC_i = decode_float(cp,optlen);
      break;
    case DC_Q_OFFSET:
      demod->sdr.DC_q = decode_float(cp,optlen);
      break;
    case IQ_IMBALANCE:
      demod->sdr.imbalance = decode_float(cp,optlen);
      break;
    case IQ_PHASE:
      demod->sdr.sinphi = decode_float(cp,optlen);
      break;
    case CALIBRATE:
      demod->sdr.calibration = decode_double(cp,optlen);
      break;



    case LOW_EDGE:
      demod->filter.low = decode_float(cp,optlen);
      break;
    case HIGH_EDGE:
      demod->filter.high = decode_float(cp,optlen);
      break;
    case KAISER_BETA:
      demod->filter.kaiser_beta = decode_float(cp,optlen);
      break;
    case FILTER_BLOCKSIZE:
      demod->filter.L = decode_int(cp,optlen);
      break;
    case FILTER_FIR_LENGTH:
      demod->filter.M = decode_int(cp,optlen);
      break;
    case NOISE_BANDWIDTH:
      Noise_bandwidth = decode_float(cp,optlen);
      break;
    case IF_POWER:
      demod->sig.if_power = decode_float(cp,optlen);
      break;
    case BASEBAND_POWER:
      demod->sig.bb_power = decode_float(cp,optlen);
      break;
    case NOISE_DENSITY:
      demod->sig.n0 = decode_float(cp,optlen);
      break;
    case DEMOD_TYPE:
      demod->demod_type = decode_int(cp,optlen); // ????
      break;
    case INDEPENDENT_SIDEBAND:
      demod->filter.isb = decode_int(cp,optlen);
      break;
    case DEMOD_SNR:
      demod->sig.snr = decode_float(cp,optlen);
      break;
    case GAIN:
      demod->agc.gain = decode_float(cp,optlen);
      break;
    case HEADROOM:
      demod->agc.headroom = decode_float(cp,optlen);
      break;
    case AGC_HANGTIME:
      demod->agc.hangtime = decode_float(cp,optlen);
      break;
    case AGC_RECOVERY_RATE:
      demod->agc.recovery_rate = decode_float(cp,optlen);
      break;
    case AGC_ATTACK_RATE:
      demod->agc.attack_rate = decode_float(cp,optlen);
      break;
    case FREQ_OFFSET:
      demod->sig.foffset = decode_float(cp,optlen);
      break;
    case PEAK_DEVIATION:
      demod->sig.pdeviation = decode_float(cp,optlen);
      break;
    case PL_TONE:
      demod->sig.plfreq = decode_float(cp,optlen);
      break;
    case PLL_LOCK:
      demod->sig.pll_lock = decode_int(cp,optlen);
      break;
    case PLL_ENABLE:
      demod->opt.pll = decode_int(cp,optlen);
      break;
    case PLL_SQUARE:
      demod->opt.square = decode_int(cp,optlen);
      break;
    case PLL_PHASE:
      demod->sig.cphase = decode_float(cp,optlen);
      break;
    case OUTPUT_CHANNELS:
      demod->output.channels = decode_int(cp,optlen);
      break;
    case OUTPUT_LEVEL:
      demod->output.level = decode_float(cp,optlen);
      break;
    default:
      break;
    }
    cp += optlen;
  }
 done:;
}
// Set major operating mode
// Adapted from the version in radio,c, but simpler
int preset_mode(struct demod * const demod,const char * const mode){
  assert(demod != NULL);
  if(demod == NULL)
    return -1;

  struct modetab *mp;
  for(mp = &Modes[0]; mp < &Modes[Nmodes]; mp++){
    if(strcasecmp(mode,mp->name) == 0)
      break;
  }
  if(mp == &Modes[Nmodes])
    return -1; // Unregistered mode

  if(mp->low > mp->high){
    demod->filter.low = mp->high;
    demod->filter.high = mp->low;
  } else {
    demod->filter.low = mp->low;
    demod->filter.high = mp->high; 
  }
  demod->tune.freq += mp->shift - demod->tune.shift; // Keep audio pitch the same
  demod->tune.shift = mp->shift;
  demod->opt.flat = mp->flat;
  demod->filter.isb = mp->isb;
  demod->output.channels = mp->channels;
  demod->opt.pll = mp->pll;
  demod->opt.square = mp->square;
  demod->agc.attack_rate = mp->attack_rate;
  demod->agc.recovery_rate = mp->recovery_rate;
  demod->agc.hangtime = mp->hangtime;
  demod->opt.env = mp->env;
  demod->demod_type = mp->demod_type;
  return 0;
}      


// Adapted from the one in radio_status.c
void decode_sdr_status(struct demod *demod,unsigned char *buffer,int length){
  unsigned char *cp = buffer;
  double nfreq = NAN;
  int nsamprate = 0;

  while(cp - buffer < length){
    enum status_type type = *cp++; // increment cp to length field

    if(type == EOL)
      break; // End of list

    unsigned int optlen = *cp++;
    if(cp - buffer + optlen >= length)
      break; // Invalid length
    switch(type){
    case EOL: // Shouldn't get here since it's checked above
      goto done;
    case OUTPUT_DATA_DEST_SOCKET:
      // SDR data destination address (usually multicast)
      decode_socket(&demod->input.data_dest_address,cp,optlen);
      break;
    case RADIO_FREQUENCY:
      nfreq = decode_double(cp,optlen);
      break;
    case OUTPUT_SAMPRATE:
      nsamprate = decode_int(cp,optlen);
      if(nsamprate != demod->sdr.status.samprate){
	demod->input.samprate = demod->sdr.status.samprate = nsamprate;
	demod->filter.decimate = demod->sdr.status.samprate / demod->output.samprate;
      }
      break;
    case COMMANDS:
      demod->input.commands = decode_int(cp,optlen);
      break;
    case AD_LEVEL:
      demod->sdr.ad_level = decode_float(cp,optlen);
      break;
    case GPS_TIME:
      demod->sdr.status.timestamp = decode_int(cp,optlen);
      break;
    case LOW_EDGE:
      demod->sdr.min_IF = decode_float(cp,optlen);
      break;
    case HIGH_EDGE:
      demod->sdr.max_IF = decode_float(cp,optlen);
      break;
    case LNA_GAIN:
      demod->sdr.status.lna_gain = decode_int(cp,optlen);
      break;
    case MIXER_GAIN:
      demod->sdr.status.mixer_gain = decode_int(cp,optlen);
      break;
    case IF_GAIN:
      demod->sdr.status.if_gain = decode_int(cp,optlen);
      break;
    case GAIN:
      demod->sdr.gain_factor = powf(10.,-0.05*decode_float(cp,optlen));
      break;
    case DC_I_OFFSET:
      demod->sdr.DC_i = decode_float(cp,optlen);
      break;
    case DC_Q_OFFSET:
      demod->sdr.DC_q = decode_float(cp,optlen);
      break;
    case IQ_IMBALANCE:
      demod->sdr.imbalance = decode_float(cp,optlen);
      break;
    case IQ_PHASE:
      demod->sdr.sinphi = decode_float(cp,optlen);
      break;
    case CALIBRATE:
      demod->sdr.calibration = decode_double(cp,optlen);
      break;
    default:
      break;
    }
    cp += optlen;
  }
  if(!isnan(nfreq) && demod->sdr.status.frequency != nfreq && demod->sdr.status.samprate != 0){
    // Recalculate LO2
    demod->sdr.status.frequency = nfreq;
  }
  done:;
}
