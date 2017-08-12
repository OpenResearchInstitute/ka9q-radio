// $Id: modes.c,v 1.11 2017/08/10 10:48:35 karn Exp karn $
#include <limits.h>
#include <stdio.h>
#include <bsd/string.h>
#include <string.h>
#include <errno.h>
#include "radio.h"
#include "dsp.h"


struct modetab Modes[256];
int Nmodes;

extern char Libdir[];

// Linkage table from ascii names to demodulator routines
struct demodtab {
  char name[16];
  void * (*demod)(void *);
} Demodtab[] = {
  {"fm",  demod_fm},
  {"am",  demod_am},
  {"cam", demod_cam},
  {"ssb", demod_ssb},
  {"dsb", demod_dsb},
  {"iq",  demod_iq}
};
#define NDEMOD (sizeof(Demodtab)/sizeof(struct demodtab))

int readmodes(char *file){
  char pathname[PATH_MAX];
  snprintf(pathname,sizeof(pathname),"%s/%s",Libdir,file);
  FILE * const fp = fopen(pathname,"r");
  if(fp == NULL){
    fprintf(stderr,"Can't read mode table %s:%s\n",pathname,strerror(errno));
    return -1;
  }
  char line[PATH_MAX];
  while(fgets(line,sizeof(line),fp) != NULL){
    chomp(line);
    if(line[0] == '#' || line[0] == '*' || line[0] == '/')
      continue; // comment

    char name[16],demod[16],options[16];
    float low,high,dial;
    if(sscanf(line,"%16s %16s %f %f %f %16s",name,demod,&low,&high,&dial,options) < 4)
      continue; // Too short, or in wrong format

    int i;
    for(i=0;i<NDEMOD;i++)
      if(strncasecmp(demod,Demodtab[i].name,strlen(Demodtab[i].name)) == 0)
	break;
      
    if(i == NDEMOD)
      continue; // Not found
    
    strlcpy(Modes[Nmodes].name,name,sizeof(Modes[Nmodes].name));
    Modes[Nmodes].demod = Demodtab[i].demod;
    Modes[Nmodes].low = low;
    Modes[Nmodes].high = high;
    Modes[Nmodes].dial = dial;
    Modes[Nmodes].flags = 0;
    if(strcasecmp(options,"conj") == 0){
      Modes[Nmodes].flags |= CONJ;
    } else if(strcasecmp(options,"flat") == 0){
      Modes[Nmodes].flags |= FLAT;
    }
    Nmodes++;
  }
  return 0;
}

