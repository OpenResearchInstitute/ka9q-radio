// $Id: display.c,v 1.94 2017/10/16 13:22:29 karn Exp karn $
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
#if defined(linux)
#include <bsd/string.h>
#endif
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
  WINDOW *pwin = newwin(5,90,15,0);
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

FILE *Tty;
SCREEN *Term;


void display_cleanup(void *arg){
  echo();
  nocbreak();
  endwin();
  if(Term)
    delscreen(Term);
  if(Tty)
    fclose(Tty);
}

// Adjust the selected item up or down one step
static void adjust_item(struct demod *demod,const int tuneitem,const double tunestep){
  switch(tuneitem){
  case 0: // Carrier frequency
  case 1: // Center frequency - treat the same
    if(!demod->frequency_lock) // Ignore if locked
      set_freq(demod,get_freq(demod) + tunestep,NAN);
    break;
  case 2: // First LO
    if(fabs(tunestep) < 1)
      break; // First LO can't make steps < 1  Hz
    
    if(demod->frequency_lock){
      // Keep frequency but lower LO2, which will increase LO1
      double new_lo2 = demod->second_LO - tunestep;
      if(LO2_in_range(demod,new_lo2,0))
	set_freq(demod,get_freq(demod),new_lo2);
    } else {
      // Retune radio but keep IF the same
      set_freq(demod,get_freq(demod) + tunestep,demod->second_LO);
    }
    break;
  case 3: // IF
    ; // needed because next line is declaration
    double new_lo2 = demod->second_LO - tunestep;
    if(LO2_in_range(demod,new_lo2,0)){ // Ignore if out of range
      if(demod->frequency_lock){
	set_freq(demod,get_freq(demod),new_lo2);
      } else {
	// Vary RF and IF together to keep LO1 the same
	set_freq(demod,get_freq(demod) + tunestep,new_lo2);
      }
    }
    break;
  case 4: // Filter low edge
    demod->low += tunestep;
    set_filter(demod->filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);
    break;
  case 5: // Filter high edge
    demod->high += tunestep;
    set_filter(demod->filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);
    break;
  case 6: // Post-detection audio frequency shift
    set_shift(demod,demod->shift + tunestep);
    break;
  case 7: // Kaiser window beta parameter for filter
    if(demod->kaiser_beta + tunestep >= 0.0){
      demod->kaiser_beta += tunestep;
      set_filter(demod->filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);
    }
    break;
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

  // talk directly to the terminal
  Tty = fopen("/dev/tty","r+");
  Term = newterm(NULL,Tty,Tty);
  set_term(Term);
  keypad(stdscr,TRUE);
  timeout(Update_interval); // update interval when nothing is typed
  cbreak();
  noecho();

  int row = 0;
  int col = 0;

  WINDOW * const tuning = newwin(8,35,row,col);    // Frequency information
  col += 35;
  WINDOW * const info = newwin(8,42,row,col);     // Band information

  col = 0;
  row += 8;
  WINDOW * const sig = newwin(8,25,row,col); // Signal information
  col += 25;
  WINDOW * const filtering = newwin(12,22,row,col);
  col += 22;
  WINDOW * const demodulator = newwin(12,25,row,col);
  col += 25;
  WINDOW * const sdr = newwin(12,25,row,col); // SDR information
  col = 0;
  row += 12;
  WINDOW * const network = newwin(8,78,row,col); // Network status information
  row += 8;
  WINDOW * const misc = newwin(4,78,row,col);
  int const dial_fd = open(DIAL,O_RDONLY|O_NDELAY);  // Powermate knob?

  struct sockaddr old_input_source_address;
  char source[NI_MAXHOST];
  char sport[NI_MAXSERV];
  memset(source,0,sizeof(source));
  memset(sport,0,sizeof(sport));

  for(;;){

    // Update display

    // Frequency control section - these can be adjusted by the user
    // using the keyboard or tuning knob, so be careful with formatting
    wmove(tuning,0,0);
    wclrtobot(tuning);    
    int row = 0;
    mvwprintw(tuning,++row,1,"Carrier%'21.3f Hz",get_freq(demod));

    if(demod->frequency_lock)
      mvwchgat(tuning,row,15,20,A_UNDERLINE,0,NULL);

    mvwprintw(tuning,++row,1,"Center%'22.3f Hz",get_freq(demod) + (demod->high + demod->low)/2);
    // second LO frequency is negative of IF, i.e., a signal at +48 kHz
    // needs a second LO frequency of -48 kHz to bring it to zero
    mvwprintw(tuning,++row,1,"First LO%'20.3f Hz",get_first_LO(demod));
#if 0
    if(!LO2_in_range(demod,demod->second_LO,1)){
      // LO2 is near its edges where signals from the opposite edge
      // get aliased; warn about this
      double alias;
      if(demod->second_LO > 0)
	alias = get_first_LO(demod) - demod->second_LO + demod->samprate;
      else
	alias = get_first_LO(demod) - demod->second_LO - demod->samprate;	
      wprintw(tuning," alias %'.3f Hz",alias);
    }
#endif
    mvwprintw(tuning,++row,1,"IF%'26.3f Hz",-demod->second_LO);
    {
      double d = get_doppler(demod);
      if(d != 0){
	mvwprintw(tuning,++row,1,"Doppler%'21.3f Hz",d);
	mvwprintw(tuning,++row,1,"Dop rate%20.3f Hz/s",get_doppler_rate(demod));
      }
    }
    box(tuning,0,0);
    mvwprintw(tuning,0,5,"Tuning");
    wnoutrefresh(tuning);

    // Display ham band emission data, if available
    wmove(info,0,0);
    wclrtobot(info);
    row = 1;

    mvwprintw(info,row++,1,"Receiver profile: %s",demod->mode);

    if(demod->doppler_command)
      mvwprintw(info,row++,1,"Doppler: %s",demod->doppler_command);
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

      mvwprintw(info,row++,1,"Band: %s",bp_low->name);

      if(intersect.modes){
	mvwprintw(info,row++,1,"Emissions: ");
	if(intersect.modes & CW)
	  wprintw(info,"CW ");
	if(intersect.modes & DATA)
	  wprintw(info,"Data ");
	if(intersect.modes & VOICE)
	  wprintw(info,"Voice ");
	if(intersect.modes & IMAGE)
	  wprintw(info,"Image");
      }
      if(intersect.classes){
	mvwprintw(info,row++,1,"Privs: ");
	if(intersect.classes & EXTRA_CLASS)
	  wprintw(info,"Extra ");
	if(intersect.classes & ADVANCED_CLASS)
	  wprintw(info,"Adv ");
	if(intersect.classes & GENERAL_CLASS)
	  wprintw(info,"Gen ");
	if(intersect.classes & TECHNICIAN_CLASS)
	  wprintw(info,"Tech ");
	if(intersect.classes & NOVICE_CLASS)
	  wprintw(info,"Nov ");
      }
    }

    box(info,0,0);
    mvwprintw(info,0,5,"Info");
    wnoutrefresh(info);
    

    wmove(filtering,0,0);
    wclrtobot(filtering);
    int const N = demod->L + demod->M - 1;
    row = 1;
    mvwprintw(filtering,row++,1,"low  %'+10.3f Hz",demod->low);
    mvwprintw(filtering,row++,1,"high %'+10.3f Hz",demod->high);
    mvwprintw(filtering,row++,1,"shift%'+10.3f Hz",demod->shift);
    mvwprintw(filtering,row++,1,"beta %'10.3f",demod->kaiser_beta);
    mvwprintw(filtering,row++,1,"block%'10d samp",demod->L);
    mvwprintw(filtering,row++,1,"FIR  %'10d samp",demod->M);
    mvwprintw(filtering,row++,1,"bin  %'10.3f Hz",demod->samprate / N);
    mvwprintw(filtering,row++,1,"delay%'10.3f s",(N - (demod->M - 1)/2)/demod->samprate);
    mvwprintw(filtering,row++,1,"rate %d/%d",demod->interpolate,demod->decimate);
    box(filtering,0,0);
    mvwprintw(filtering,0,5,"Filtering");
    wnoutrefresh(filtering);

    // Signal data
    wmove(sig,0,0);
    wclrtobot(sig);

    float bw = 0;
    if(demod->filter != NULL)
      bw = demod->samprate * demod->filter->noise_gain;
    float sn0 = demod->bb_power / demod->n0 - bw;

    row = 1;
    mvwprintw(sig,row++,1,"IF%13.1f dBFS",power2dB(demod->if_power));
    mvwprintw(sig,row++,1,"Baseband%7.1f dBFS",power2dB(demod->bb_power));
    mvwprintw(sig,row++,1,"N0      %7.1f dBFS/Hz",power2dB(demod->n0));
    mvwprintw(sig,row++,1,"S/N0    %7.1f dBHz",10*log10f(sn0));
    mvwprintw(sig,row++,1,"NBW     %7.1f dBHz",10*log10f(bw));
    mvwprintw(sig,row++,1,"SNR     %7.1f dB",10*log10f(sn0/bw));
    box(sig,0,0);
    mvwprintw(sig,0,5,"Signal");
    wnoutrefresh(sig);

    // Demodulator info
    wmove(demodulator,0,0);
    wclrtobot(demodulator);
    row = 1;


    // Display these only if they're in use by the current mode
    if(!isnan(demod->snr))
      mvwprintw(demodulator,row++,1,"Loop SNR%11.1f dB",power2dB(demod->snr));

    mvwprintw(demodulator,row++,1,"AF Gain%12.1f dB",voltage2dB(demod->gain));
    
    if(demod->foffset != 0)
      mvwprintw(demodulator,row++,1,"offset%'+13.1f Hz",demod->foffset);

    if(!isnan(demod->pdeviation))
      mvwprintw(demodulator,row++,1,"deviat%13.1f Hz",demod->pdeviation);

    if(!isnan(demod->cphase))
      mvwprintw(demodulator,row++,1,"phase%+14.1f deg",demod->cphase*DEGPRA);

    if(!isnan(demod->plfreq))
      mvwprintw(demodulator,row++,1,"tone%15.1f Hz",demod->plfreq);

    if(!isnan(demod->spare))
      mvwprintw(demodulator,row++,1,"spare%14.1f   ",demod->spare);      

    wmove(demodulator,row,0);
    if(demod->flags & ENVELOPE)
      wprintw(demodulator,"Envelope"); // arguably not linear
    if(demod->flags & CAL)
      wprintw(demodulator,"%s"," Cal"); 
    if(demod->flags & CONJ) 
      wprintw(demodulator,"%s"," ISB"); 
    if(demod->flags & FLAT) 
      wprintw(demodulator,"%s"," Flat"); 
    if(demod->flags & SQUARE) 
      wprintw(demodulator,"%s"," Squaring"); 
    if(demod->flags & COHERENT) 
      wprintw(demodulator,"%s"," PLL"); 
    if(demod->flags & MONO) 
      wprintw(demodulator,"%s"," Mono"); 
    else 
      wprintw(demodulator,"%s"," Stereo");


    box(demodulator,0,0);
    mvwprintw(demodulator,0,5,"%s demodulator",demod->demod_name);
    wnoutrefresh(demodulator);
    

    // SDR hardware status: sample rate, tcxo offset, I/Q offset and imbalance, gain settings
    wmove(sdr,0,0);
    wclrtobot(sdr);
    row = 1;
    mvwprintw(sdr,row++,1,"Samprate%'10d Hz",demod->status.samprate); // Nominal
    mvwprintw(sdr,row++,1,"LO%'16.0f Hz",demod->status.frequency); // Integer for now (SDR dependent)
    mvwprintw(sdr,row++,1,"TCXO cal%'+10.3f ppm",demod->calibrate *1e6);
    mvwprintw(sdr,row++,1,"I offset%+10.6f",demod->DC_i);  // Scaled to +/-1
    mvwprintw(sdr,row++,1,"Q offset%+10.6f",demod->DC_q);
    mvwprintw(sdr,row++,1,"I/Q imbal%+9.3f dB",power2dB(demod->imbalance));
    mvwprintw(sdr,row++,1,"I/Q phi%+11.1f deg",demod->sinphi*DEGPRA);
    mvwprintw(sdr,row++,1,"LNA%15u",demod->status.lna_gain);   // SDR dependent
    mvwprintw(sdr,row++,1,"Mix gain%10u",demod->status.mixer_gain); // SDR dependent
    mvwprintw(sdr,row++,1,"IF gain%11u dB",demod->status.if_gain); // SDR dependent
    box(sdr,0,0);
    mvwprintw(sdr,0,5,"SDR Hardware");
    wnoutrefresh(sdr);

    // Network status
    if(memcmp(&old_input_source_address,&demod->input_source_address,sizeof(old_input_source_address)) != 0){
      // First time, or source has changed
      memcpy(&old_input_source_address,&demod->input_source_address,sizeof(old_input_source_address));
      getnameinfo((struct sockaddr *)&demod->input_source_address,sizeof(demod->input_source_address),
		  source,sizeof(source),
		  sport,sizeof(sport),NI_NOFQDN|NI_DGRAM|NI_NUMERICHOST);
    }
    row = 1;
    extern int Delayed,Skips;
    wmove(network,0,0);
    wclrtobot(network);
    mvwprintw(network,row++,1,"Source %s:%s -> %s",source,sport,demod->iq_mcast_address_text);
    mvwprintw(network,row++,1,"IQ pkts %'llu; late %'d; skips %'d",demod->iq_packets,
	      Delayed,Skips);
    if(audio->filename != NULL)
      mvwprintw(network,row++,1,"PCM stream %s",audio->filename);

    if(audio->opus_bitrate != 0 || audio->rtp_pcm != 0) 
      // Audio output dest address (usually multicast)
      mvwprintw(network,row++,1,"Sink %s",audio->audio_mcast_address_text);

    if(audio->opus_bitrate > 0)
      mvwprintw(network,row++,1,"Opus %.1fms%s %.0f kb/s; %'.1f kb/s; pkts %'llu",
		audio->opus_blocktime,audio->opus_dtx ? " dtx":"",
		audio->opus_bitrate/1000.,audio->bitrate/1000.,audio->audio_packets);

    if(audio->rtp_pcm)
      mvwprintw(network,row++,1,"PCM %'d Hz; %'.1f kb/s; pkts %'llu",audio->samprate,audio->bitrate/1000.,
		audio->audio_packets);

    box(network,0,0);
    mvwprintw(network,0,5,"Network");
    wnoutrefresh(network);

    // A message from our sponsor...
    row = 1;
    wmove(misc,0,0);
    wclrtobot(misc);
    mvwprintw(misc,row++,1,"KA9Q SDR Receiver v1.0; Copyright 2017 Phil Karn");
    mvwprintw(misc,row++,1,"Compiled on %s at %s",__DATE__,__TIME__);
    box(misc,0,0);
    wnoutrefresh(misc);
    

    // Highlight cursor for tuning step
    // A little messy because of the commas in the frequencies
    // They come from the ' option in the printf formats
    int hcol;
    if(demod->tunestep >= -3 && demod->tunestep <= -1){ // .001 or .01 or .1
      hcol = - demod->tunestep + 1;
    } else if(demod->tunestep >= 0 && demod->tunestep <= 2){
      hcol = - demod->tunestep;  // 1, 10, 100
    } else if(demod->tunestep >= 3 && demod->tunestep <= 5){
      hcol = - demod->tunestep - 1; // 1,000; 10,000; 100,000
    } else if(demod->tunestep >= 6 && demod->tunestep <= 8){
      hcol = - demod->tunestep - 2; // 1,000,000; 10,000,000; 100,000,000
    } else if(demod->tunestep >= 9 && demod->tunestep <= 9){
      hcol = - demod->tunestep - 3; // 1,000,000,000
    } else
      hcol = 0; // can't happen, but shuts up compiler
    // Highlight digit
    if(tuneitem >= 0 && tuneitem <= 3){
      mvwchgat(tuning,tuneitem+1,hcol+24,1,A_STANDOUT,0,NULL);
      wnoutrefresh(tuning);
    }
    else if(tuneitem >= 4 && tuneitem <= 7){
      mvwchgat(filtering,tuneitem+1-4,hcol+11,1,A_STANDOUT,0,NULL);
      wnoutrefresh(filtering);
    }
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
    if(dial_fd != -1 && read(dial_fd,&event,sizeof(event)) == sizeof(event)){
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
      goto loopend; // Start again with display refresh
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

	int const i = setup_mcast(str,0);
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
	strlcpy(demod->iq_mcast_address_text,str,sizeof(demod->iq_mcast_address_text));
	// Reset error counts
	Skips = Delayed = 0;
      }
      break;
    case 'l': // Toggle RF tuning lock; affects how adjustments to LO and IF behave
      demod->frequency_lock = !demod->frequency_lock;
      break;
    case KEY_NPAGE: // Page Down/tab key
    case '\t':      // go to next tuning item
      tuneitem = (tuneitem + 1) % 8;
      break;
    case KEY_BTAB:  // Page Up/Backtab, i.e., shifted tab:
    case KEY_PPAGE: // go to previous tuning item
      tuneitem = (8 + tuneitem - 1) % 8;
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
    case 'M':   // Select demod mode from list. Note: overwrites filters
    case 'm':   // keep filters the same when changing modes
      {
	char str[1024];
	snprintf(str,sizeof(str),"Enter mode %s [ ",c == 'M' ? "(filter reset)": "");
	for(int i=0;i < Nmodes;i++){
	  strlcat(str,Modes[i].name,sizeof(str) - strlen(str) - 1);
	  strlcat(str," ",sizeof(str) - strlen(str) - 1);
	}
	strlcat(str,"]: ",sizeof(str) - strlen(str) - 1);
	getentry(str,str,sizeof(str));
	if(strlen(str) > 0){
	  set_mode(demod,str,c == 'M');
	}
      }
      break;
    case 'f':   // Tune to new frequency
      {
	char str[160];
	getentry("Enter carrier frequency: ",str,sizeof(str));
	double const f = parse_frequency(str); // Handles funky forms like 147m435
	if(f > 0){
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
      }
      break;
    case 'i':    // Recenter IF to +/- samprate/4
      set_freq(demod,get_freq(demod),demod->samprate/4);
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
	  set_filter(demod->filter,demod->samprate/demod->decimate,demod->low,demod->high,demod->kaiser_beta);
	} else
	  beep();
      }
      break;
    default:
      beep();
      break;
    }
  loopend:;
    doupdate();
  }
 done:;
  endwin();
  set_term(NULL);
  if(Term != NULL)
    delscreen(Term);
  if(Tty != NULL)
    fclose(Tty);
  
  // Dump receiver state to default
  savestate(demod,"default");
  audio_cleanup(demod->audio);
  exit(0);
}
