#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "bandplan.h"


struct bandplan Bandplans[100];
int Nbandplans;


static int compar(const void *a,const void *b){
  const double f = *(double *)a;
  const struct bandplan *bp = b;

  if(f < bp->lower)
    return -1;
  if(f > bp->upper)
    return 1;
  else
    return 0;
}

int Bandplan_init;
extern int init_bandplan(int class);

struct bandplan *lookup_frequency(double f){
  f /= 1e6;

  if(!Bandplan_init){
    init_bandplan(EXTRA_CLASS);
    Bandplan_init = 1;
  }
  return bsearch(&f,Bandplans,Nbandplans,sizeof(struct bandplan),compar);
}


int init_bandplan(int class){
  char line[160];
  char classes[160];
  char modes[160];
  char name[160];
  double power;
  double lower,upper;
  FILE *bandplan;

  int i=0,r;

  if((bandplan = fopen("bandplan.txt","r")) == NULL)
    return -1;

  memset(line,0,sizeof(line));
  while(fgets(line,sizeof(line),bandplan) != NULL){
    if(line[0] == ';' || line[0] == '#')
      continue;
    for(r=0;r<strlen(line);r++)
      line[r] = tolower(line[r]);

    r = sscanf(line,"%lf %lf %s %s %s %lf",&lower,&upper,classes,modes,name,&power);
	   
    if(r < 5)
      continue;

    memset(&Bandplans[i],0,sizeof(struct bandplan));
    Bandplans[i].lower = lower;
    Bandplans[i].upper = upper;
    char *cp;
    for(cp = classes;*cp != '\0';cp++)
      switch(*cp){
      case 'e':
	Bandplans[i].classes |= EXTRA_CLASS;
	break;
      case 'a':
	Bandplans[i].classes |= ADVANCED_CLASS;
	break;
      case 'g':
	Bandplans[i].classes |= GENERAL_CLASS;
	break;
      case 't':
	Bandplans[i].classes |= TECHNICIAN_CLASS;
	break;
      case 'n':
	Bandplans[i].classes |= NOVICE_CLASS;
	break;
      }

    for(cp = modes;*cp != '\0';cp++)
      switch(*cp){
      case 'c':
	Bandplans[i].modes |= CW;
	break;
      case 'v':
	Bandplans[i].modes |= VOICE;
	break;
      case 'i':
	Bandplans[i].modes |= IMAGE;
	break;
      case 'd':
	Bandplans[i].modes |= DATA;
	break;
      }
    
    strncpy(Bandplans[i].name,name,sizeof(Bandplans[i].name));
    
    if(r >= 6)
      Bandplans[i].power = power;

    if(Bandplans[i].classes & class)
      i++;
  }
  fprintf(stderr,"%d entries read\n",i);
  Nbandplans = i;
  return 0;
}
#if 0
int main(){
  double f;
  struct bandplan *bp;

  while(1){
    scanf("%lf",&f);
    bp = lookup_frequency(f);
    if(bp != NULL){
      printf("%lf: %s",f,bp->name);
      if(bp->modes & CW)
	printf(" CW");
      if(bp->modes & DATA)
	printf(" Data");
      if(bp->modes & VOICE)
	printf(" Voice");
      if(bp->modes & IMAGE)
	printf(" Image");
      if(bp->power != 0)
	printf(" %.0lf watts\n",bp->power);

      printf("\n");
    }
  }
}

#endif