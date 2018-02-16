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
  unsigned char packet[2048];
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
      int i;
      for(i = 0; i < len;i++){
	if(packet[i] & 1)
	  break;
      }
      if(i == len){
	// Invalid
	printf("Incomplete frame\n");
	continue;
      }
      if(packet[i+1] != 0x03 || packet[i+2] != 0xf0){
	printf("Invalid ax25 type\n");
	continue;
      }
      char *data = (char *)(packet + i + 3); // First byte of text field
      // Extract lat/long

      // Parse APRS position packets
      // The APRS spec is an UNBELIEVABLE FUCKING MESS THAT SHOULD BE SHOT, SHREDDED, BURNED AND SENT TO HELL!
      // There, now I feel a little better. But not much.
      double latitude=NAN,longitude=NAN,altitude=NAN,timestamp = NAN;
      int t, hours=-1, minutes=-1,days=-1,seconds=-1;

      // Sample WB8ELK LU1ESY-3>APRS,TCPIP*,qAS,WB8ELK:/180205h3648.75S/04627.50WO000/000/A=039566 2 4.50 25 12060 GF63SE 1N7MSE 226
      // Sample PITS "!/%s%sO   /A=%06ld|%s|%s/%s,%d'C,http://www.pi-in-the-sky.com",

      if(*data == '/' || *data == '@'){
	// process timestamp
	char *ncp = NULL;
	data++;
	t = strtol(data,&ncp,10);
	if(*ncp == 'h'){
	  // Hours, minutes, seconds
	  days = 0;
	  hours = t / 10000;
	  t -= hours * 10000;
	  minutes = t / 100;
	  t -= minutes * 100;
	  seconds = t;
	} else if(*ncp == 'z'){
	  // day, hours minutes zulo
	  days = t / 10000;
	  t -= days * 10000;
	  hours = t / 100;
	  t -= hours * 100;
	  minutes = t;
	  seconds = 0;
	} else if(*ncp == '/'){
	  // day, hours, minutes local -- HOW AM I SUPPOSED TO KNOW THE TIME ZONE ??!?!?
	  days = t / 10000;
	  t -= days * 10000;
	  hours = t / 100;
	  t -= hours * 100;
	  minutes = t;
	  seconds = 0;
	}

	data = ncp+1; // skip 'h' or 'z' (process?)
      } else if(*data == '!' || *data == '='){
	// Position without timestamp
	data++;
      } else {
	printf("Unsupported APRS frame type 0x%x (%c)\n",*data,*data);
	continue;
      }
	      
      // parse position
      if(*data == '/'){
	// Compressed
	data++; // skip /
	latitude = 90 - decode_base91(data)/380926.;
	longitude = -180 + decode_base91(data+4) / 190463.;
	data += 12;
      } else {
	// Uncompressed
	char *ncp = NULL;
	latitude = strtod(data,&ncp) / 100.;
	latitude = (int)(latitude) + fmod(latitude,1.0) / 0.6;
	if(tolower(*ncp) == 's')
	  latitude = -latitude;
	data = ncp + 2; // Skip S and /
	longitude = strtod(data,&ncp) / 100.;
	longitude = (int)(longitude) + fmod(longitude,1.0) / 0.6;
	if(tolower(*ncp) == 'w')
	  longitude = -longitude;
	data = ncp + 2; // Skip W and /
	// Look for A=
	while(*data != '\0' && *(data+1) != '\0'){
	  if(*data == 'A' && data[1] == '='){
	    altitude = strtol(data+2,&ncp,10);
	    break;
	  } else
	    data++;
	}
	altitude *= 0.3048; // Convert to meters
      }
      printf("days %d hours %d minutes %d seconds %d\n",days,hours,minutes,seconds);
      printf("Latitude %.6lf deg; Longitude %.6lf deg; Altitude %.1lf m\n",latitude,longitude,altitude);
    }
  }
}

