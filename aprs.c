// $Id: aprs.c,v 1.9 2018/07/06 06:06:12 karn Exp karn $
// Process AX.25 frames containing APRS data, extract lat/long/altitude, compute az/el
// INCOMPLETE, doesn't yet drive antenna rotors
// Should also use RTP for AX.25 frames
// Should get station location from a GPS receiver
// Copyright 2018, Phil Karn, KA9Q

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
#include <math.h>
#include <time.h>

#include "misc.h"
#include "dsp.h"
#include "multicast.h"
#include "ax25.h"

char *Mcast_address_text = "ax25.vhf.mcast.local:8192";
char *Source = NULL;

double const WGS84_E = 0.081819190842622;  // Eccentricity
double const WGS84_A = 6378137;         // Equatorial radius, meters

int Verbose;
int Input_fd = -1;

#define square(x) ((x)*(x))

int main(int argc,char *argv[]){
  setlocale(LC_ALL,getenv("LANG"));
  double latitude,longitude,altitude;


#if 0
  // Use defaults - KA9Q location, be sure to change elsewhere!!
  latitude = 32.8604;
    longitude = -117.1889;
    altitude = 0;
#else
    // MCHSARC
    latitude = 32.967233;
    longitude = -117.122382;
    altitude = 200;
#endif
    
  int c;
  while((c = getopt(argc,argv,"L:M:A:I:vs:")) != EOF){
    switch(c){
    case 'L':
      latitude = strtod(optarg,NULL);
      break;
    case 'M':
      longitude = strtod(optarg,NULL);
      break;
    case 'A':
      altitude = strtod(optarg,NULL);
      break;
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
      fprintf(stderr,"Usage: %s [-L latitude] [-M longitude] [-A altitude] [-a | -s sourcecall] [-v] [-I mcast_address]\n",argv[0]);
      fprintf(stderr,"Defaults: %s -L %lf -M %lf -A %lf -s %s -I %s\n",argv[0],
	      latitude,longitude,altitude,Source,Mcast_address_text);
      exit(1);
    }
  }
  printf("APRS az/el program by KA9Q\n");
  if(Source){
    printf("Watching for %s\n",Source);
  } else {
    printf("Watching all stations\n");
  }
  printf("Station coordinates: longitude %.6lf deg; latitude %.6lf deg; altitude %.1lf m\n",
	 longitude,latitude,altitude);

  // Station position in earth-centered ROTATING coordinate system
  double station_x,station_y,station_z;
  // Unit vectors defining station's orientation
  double up_x,up_y,up_z;
  double south_x,south_y,south_z;  
  double east_x,east_y,east_z;
  
  {
    double sinlat,coslat;
    sincos(RAPDEG*latitude,&sinlat,&coslat);
    double sinlong,coslong;
    sincos(RAPDEG*longitude,&sinlong,&coslong);
    
    double tmp = WGS84_A/sqrt(1-(square(WGS84_E)*square(sinlat)));
    station_x = (tmp + altitude) * coslat * coslong;
    station_y = (tmp + altitude) * coslat * sinlong;
    station_z = (tmp*(1-square(WGS84_E)) + altitude) * sinlat;
    
    // Zenith vector is (coslong*coslat, sinlong*coslat, sinlat)
    up_x = coslong * coslat;
    up_y = sinlong * coslat;
    up_z = sinlat;

    east_x = -sinlong;
    east_y = coslong;
    east_z = 0;

    south_x = coslong*sinlat;
    south_y = sinlong*sinlat;
    south_z = -(sinlong*sinlong*sinlat + coslong*coslong*coslat);
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

  setlinebuf(stdout);

  while((len = recv(Input_fd,packet,sizeof(packet),0)) > 0){
    struct ax25_frame frame;
    //    dump_frame(packet,len);
    if(ax25_parse(&frame,packet,len) < 0){
      // Unparseable AX25 header
      continue;
    }

    // Is this the droid we're looking for?
    if(Source == NULL || strncasecmp(frame.source,Source,sizeof(frame.source)) == 0){
      time_t t;
      struct tm *tmp;
      time(&t);
      tmp = gmtime(&t);
      printf("%d %s %04d %02d:%02d:%02d UTC:",tmp->tm_mday,Months[tmp->tm_mon],tmp->tm_year+1900,
	     tmp->tm_hour,tmp->tm_min,tmp->tm_sec);

      printf(" %s",frame.source);
      if(frame.control != 0x03 || frame.type != 0xf0){
	printf(" Invalid ax25 type\n");
	continue;
      }
      char *data = frame.information; // First byte of text field
      // Extract lat/long

      // Parse APRS position packets
      // The APRS spec is an UNBELIEVABLE FUCKING MESS THAT SHOULD BE SHOT, SHREDDED, BURNED AND SENT TO HELL!
      // There, now I feel a little better. But not much.
      double latitude=NAN,longitude=NAN,altitude=NAN;
      int hours=-1, minutes=-1,days=-1,seconds=-1;

      // Sample WB8ELK LU1ESY-3>APRS,TCPIP*,qAS,WB8ELK:/180205h3648.75S/04627.50WO000/000/A=039566 2 4.50 25 12060 GF63SE 1N7MSE 226
      // Sample PITS "!/%s%sO   /A=%06ld|%s|%s/%s,%d'C,http://www.pi-in-the-sky.com",

      int altitude_known = 0;
      if(*data == 0x60){
	// MIC-E format: latitude is in dest callsign (!!)
	{
	  int deg = (frame.dest[0] & 0xf) * 10 + (frame.dest[1] & 0xf);
	  int minutes = (frame.dest[2] & 0xf) * 10 + (frame.dest[3] & 0xf);
	  int hun_mins = (frame.dest[4] & 0xf) * 10 + (frame.dest[5] & 0xf);
	  latitude = deg + minutes/60. + hun_mins / 6000.;
	}
	// longitude is in I field (did I say how incredibly painfully ugly this is??)
	data++;
	{
	  int deg = *data++ - 28;
	  if(180 <= deg && deg <= 189)
	    deg -= 80;
	  else if(190 <= deg && deg <= 199)
	    deg -= 190;
	  if(frame.dest[4] & 0x40)
	    deg += 100;
	  
	  int minutes = *data++ - 28;
	  if(minutes > 60)
	    minutes -= 60;
	  int hun_mins = *data++ - 28;
	  
	  longitude = deg + minutes / 60. + hun_mins / 6000.;
	  if(frame.dest[3] & 0x40)
	    longitude = -longitude;
	}
      } else {
	if(*data == '/' || *data == '@'){
	  // process timestamp
	  char *ncp = NULL;
	  data++;
	  int t = strtol(data,&ncp,10);
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
	  printf(" Unsupported APRS frame type 0x%x (%c)\n",*data,*data);
	  continue;
	}
	// parse position
	if(*data == '/'){
	  // Compressed
	  data++; // skip /
	  latitude = 90 - decode_base91(data)/380926.;
	  longitude = -180 + decode_base91(data+4) / 190463.;
	  data += 12;
	} else if(isdigit(*data)){
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
	      altitude *= 0.3048; // Convert to meters
	      altitude_known = 1;
	      break;
	    } else
	      data++;
	  }
	} else {
	  printf(" Unparseable position report\n");
	  continue;
	}
      }
      if(days != -1 || hours != -1 || minutes != -1 || seconds != -1)
	printf(" %d %02d:%02d:%02d;",days,hours,minutes,seconds);
      printf(" Lat %.6lf Long %.6lf",latitude,longitude);
      
      if(altitude_known)
	printf(" Alt %.1lf m",altitude);
      else {
	altitude = 0;
      }
      putchar(';');

      double target_x,target_y,target_z;
      {
	double sinlat,coslat;
	sincos(RAPDEG*latitude,&sinlat,&coslat);
	double sinlong,coslong;
	sincos(RAPDEG*longitude,&sinlong,&coslong);
      
	double tmp = WGS84_A/sqrt(1-(square(WGS84_E)*square(sinlat))); // Earth radius under target
	target_x = (tmp + altitude) * coslat * coslong;
	target_y = (tmp + altitude) * coslat * sinlong;
	target_z = (tmp*(1-square(WGS84_E)) + altitude) * sinlat;
      }
      double look_x,look_y,look_z;
      look_x = target_x - station_x;
      look_y = target_y - station_y;
      look_z = target_z - station_z;      
      double range = sqrt(square(look_x)+square(look_y)+square(look_z));
      
      double south = (south_x * look_x + south_y * look_y + south_z * look_z) / range;
      double east = (east_x * look_x + east_y * look_y + east_z * look_z) / range;
      double up = (up_x * look_x + up_y * look_y + up_z * look_z) / range;
      
      double elevation = asin(up);
      double azimuth = M_PI - atan2(east,south);

      if(altitude_known)
	printf(" az %.1lf elev %.1lf range %'.1lf m\n",
	       azimuth*DEGPRA, elevation*DEGPRA,range);
      else
	printf(" az %.1lf range %'.1lf m\n",
	       azimuth*DEGPRA, range);
	
    }
  }
}

