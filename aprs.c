// Process AX.25 frames containing APRS data, extract lat/long/altitude


#define _GNU_SOURCE 1
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <locale.h>
#include <errno.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netdb.h>
#include <math.h>

#include "multicast.h"
#include "ax25.h"

int Verbose;
int Input_fd = -1;
char *Mcast_address_text = "ax25-mcast.local:8192";
char *Source = "W6SUN-4";

int main(int argc,char *argv[]){
  setlocale(LC_ALL,getenv("LANG"));

  int c;
  while((c = getopt(argc,argv,"I:vs:")) != EOF){
    switch(c){
    case 's':
      Source = optarg;
      break;
    case 'v':
      Verbose++;
      break;
    case 'I':
      Mcast_address_text = optarg;
      break;
    default:
      fprintf(stderr,"Usage: %s [-v] [-I mcast_address]\n",argv[0]);
      fprintf(stderr,"Defaults: %s -I %s\n",argv[0],Mcast_address_text);
      exit(1);
    }
  }

  // Set up multicast input
  Input_fd = setup_mcast(Mcast_address_text,0);
  if(Input_fd == -1){
    fprintf(stderr,"Can't set up input from %s\n",
	    Mcast_address_text);
    exit(1);
  }
  char packet[2048];
  int len;

  while((len = recv(Input_fd,packet,sizeof(packet),0)) > 0){
    dump_frame(packet,len);
    // Is this the droid we're looking for?
    char result[10];
    get_callsign(result,packet+7);
    printf("source = %s\n",result);
    if(1 || strncasecmp(result,Source,sizeof(result)) == 0){

      printf("match!\n");
      // Find end of address field
      char *cp = packet;
      int i;
      for(i = 0; i < len;i++){
	if(*cp++ & 1)
	  break;
      }
      if(i == len){
	// Invalid
	printf("Incomplete frame\n");
	continue;
      }
      if(cp[0] != 0x03 || cp[1] != 0xf0){
	printf("Invalid type\n");
	continue;
      }
	
      cp += 2; // Skip type and protocol fields
      cp++;
      // Extract lat/long

      double latitude,longitude,altitude=0,timestamp;
      int t, hours=-1, minutes=-1, seconds = -1;
      char *ncp;

      switch(*cp++){
      case '/': // Position with timestamp (WB8ELK pico tracker)
	// Sample WB8ELK: LU1ESY-3>APRS,TCPIP*,qAS,WB8ELK:/180205h3648.75S/04627.50WO000/000/A=039566 2 4.50 25 12060 GF63SE 1N7MSE 226
	t = strtol(cp,&ncp,10);
	hours = t / 10000;
	t -= hours * 10000;
	minutes = t / 100;
	t -= minutes * 100;
	seconds = t;
	cp = ncp+1;
	latitude = strtod(cp,&ncp) / 100.;
	latitude = (int)(latitude) + fmod(latitude,1.0) / 0.6;
	if(*ncp == 'S' || *ncp == 's')
	  latitude = -latitude;
	cp = ncp + 2;
	longitude = strtod(cp,&ncp) / 100.;
	longitude = (int)(longitude) + fmod(longitude,1.0) / 0.6;
	if(*ncp == 'w' || *ncp == 'W')
	  longitude = -longitude;
	cp = ncp;
	while(*cp != '\0' && *(cp+1) != '\0'){
	  if(*cp == 'A' && cp[1] == '='){
	    altitude = strtol(cp+2,&ncp,10);
	    break;
	  } else
	    cp++;
	}
	altitude *= 0.3048; // Convert to meters
	break;
      case '!': // Position without timestamp (Pi in the Sky)
		// "!/%s%sO   /A=%06ld|%s|%s/%s,%d'C,http://www.pi-in-the-sky.com",
	cp++; // skip /
	timestamp = 0;
	latitude = 90 - decode_base91(cp)/380926.;
	longitude = -180 + decode_base91(cp+4) / 190463.;
	cp = cp + 8;
	while(*cp != '\0' && *(cp+1) != '\0'){
	  if(*cp == 'A' && cp[1] == '='){
	    altitude = strtol(cp+2,&ncp,10);
	    break;
	  } else
	    cp++;
	}
	altitude *= 0.3048; // Convert to meters
	break;
      }
      printf("Latitude %.6lf deg; Longitude %.6lf deg; Altitude %.6lf m\n",latitude,longitude,altitude);

    }
  }


}
