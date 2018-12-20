// $Id: dump.c,v 1.6 2018/12/19 04:49:50 karn Exp karn $
#define _GNU_SOURCE 1
#include <assert.h>
#include <stdio.h>
#include <math.h>
#undef I
#include <netdb.h>

#include "misc.h"
#include "status.h"

int dump_socket(char *host,char *port,unsigned char *val,int optlen){
  struct sockaddr_storage sock;

  if(decode_socket((struct sockaddr *)&sock,val,optlen) == -1)
    return -1;
  return getnameinfo((struct sockaddr *)&sock,sizeof(sock),host,NI_MAXHOST,port,NI_MAXSERV,
	      NI_NOFQDN|NI_NUMERICHOST|NI_NUMERICSERV);
}


void dump_metadata(unsigned char *buffer,int length){
  unsigned char *cp = buffer;
  char host[NI_MAXHOST];
  char port[NI_MAXSERV];
  int i;
  char sbuf[256];

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
    case GPS_TIME:
      printf(" %s;",lltime((long long unsigned)decode_int(cp,optlen)));
      break;
    case COMMAND_TAG:
      printf(" cmd tag %llx;",(long long unsigned)decode_int(cp,optlen));
      break;
    case COMMANDS:
      printf(" commands %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case INPUT_DATA_SOURCE_SOCKET:
      dump_socket(host,port,cp,optlen);
      printf(" in data src %s:%s;",host,port);
      break;
    case INPUT_METADATA_SOURCE_SOCKET:
      dump_socket(host,port,cp,optlen);
      printf(" in metadata src %s:%s;",host,port);
      break;
    case INPUT_DATA_DEST_SOCKET:
      dump_socket(host,port,cp,optlen);      
      printf(" in data dst %s:%s;",host,port);
      break;
    case INPUT_METADATA_DEST_SOCKET:      
      dump_socket(host,port,cp,optlen);      
      printf(" in metadata dst %s:%s;",host,port);
      break;
    case INPUT_SSRC:
      printf(" in SSRC %llx;",(long long unsigned)decode_int(cp,optlen));
      break;
    case INPUT_SAMPRATE:
      printf(" in samprate %'llu Hz;",(long long unsigned)decode_int(cp,optlen));
      break;
    case INPUT_DATA_PACKETS:
      printf(" in data packets %'llu",(long long unsigned)decode_int(cp,optlen));
      break;
    case INPUT_METADATA_PACKETS:
      printf(" in metadata packets %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case INPUT_SAMPLES:
      printf(" in samples %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case INPUT_DROPS:
      printf(" in drops %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case INPUT_DUPES:
      printf(" in dupes %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case OUTPUT_DATA_SOURCE_SOCKET:
      dump_socket(host,port,cp,optlen);      
      printf(" out data src %s:%s;",host,port);
      break;
    case OUTPUT_DATA_DEST_SOCKET:
      dump_socket(host,port,cp,optlen);      
      printf(" out data dst %s:%s;",host,port);
      break;
    case OUTPUT_SSRC:
      printf(" out SSRC %llx;",(long long unsigned)decode_int(cp,optlen));
      break;
    case OUTPUT_TTL:
      printf(" out TTL %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case OUTPUT_SAMPRATE:
      printf(" out samprate %'llu Hz;",(long long unsigned)decode_int(cp,optlen));
      break;
    case OUTPUT_DATA_PACKETS:
      printf(" out data pkts %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case OUTPUT_METADATA_PACKETS:
      printf(" out metadata pkts %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case RADIO_FREQUENCY:
      printf(" RF %.3lf Hz;",decode_double(cp,optlen));
      break;
    case FIRST_LO_FREQUENCY:
      printf(" first LO %.3lf Hz;",decode_double(cp,optlen));
      break;
    case SECOND_LO_FREQUENCY:
      printf(" second LO %.3lf Hz;",decode_double(cp,optlen));
      break;
    case SHIFT_FREQUENCY:
      printf(" shift %.3lf Hz;",decode_double(cp,optlen));
      break;
    case DOPPLER_FREQUENCY:
      printf(" doppler %.3lf Hz;",decode_double(cp,optlen));
      break;
    case DOPPLER_FREQUENCY_RATE:
      printf(" doppler rate %.3lf Hz/s;",decode_double(cp,optlen));
      break;
    case LNA_GAIN:
      printf(" lna gain %'llu dB;",(long long unsigned)decode_int(cp,optlen));
      break;
    case MIXER_GAIN:
      printf(" mixer gain %'llu dB;",(long long unsigned)decode_int(cp,optlen));
      break;
    case IF_GAIN:
      printf(" if gain %'llu dB;",(long long unsigned)decode_int(cp,optlen));
      break;
    case DC_I_OFFSET:
      printf(" DC I offset %g;",decode_float(cp,optlen));
      break;
    case DC_Q_OFFSET:
      printf(" DC Q offset %g;",decode_float(cp,optlen));
      break;
    case IQ_IMBALANCE:
      printf(" gain imbal %.1f dB;",10*log10f(decode_float(cp,optlen)));
      break;
    case IQ_PHASE:
      printf(" phase imbal %.1f deg;",(180./M_PI)*asinf(decode_float(cp,optlen)));
      break;
    case LOW_EDGE:
      printf(" filt low %g Hz;",decode_float(cp,optlen));
      break;
    case HIGH_EDGE:
      printf(" filt high %g Hz;",decode_float(cp,optlen));
      break;
    case KAISER_BETA:
      printf(" filter kaiser %g;",decode_float(cp,optlen));      
      break;
    case FILTER_BLOCKSIZE:
      printf(" filter L %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case FILTER_FIR_LENGTH:
      printf(" filter M %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case NOISE_BANDWIDTH:
      printf(" noise BW %g Hz;",decode_float(cp,optlen));
      break;
    case IF_POWER:
      printf(" IF pwr %.1f dB;",decode_float(cp,optlen));
      break;
    case BASEBAND_POWER:
      printf(" BB pwr %.1f dB;",decode_float(cp,optlen));
      break;
    case NOISE_DENSITY:
      printf(" N0 %.1f dB/Hz;",decode_float(cp,optlen));
      break;
    case DEMOD_TYPE:
      i = (long long unsigned)decode_int(cp,optlen); // ????
      printf(" demod %d;",i);
      break;
    case INDEPENDENT_SIDEBAND:
      printf(" ISB %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case DEMOD_SNR:
      printf(" Demod SNR %.1f dB;",decode_float(cp,optlen));
      break;
    case GAIN:
      printf(" gain %.1f dB;",decode_float(cp,optlen));
      break;
    case FREQ_OFFSET:
      printf(" freq offset %g Hz;",decode_float(cp,optlen));
      break;
    case PEAK_DEVIATION:
      printf(" peak FM dev %g Hz;",decode_float(cp,optlen));
      break;
    case PL_TONE:
      printf(" PL tone %g Hz;",decode_float(cp,optlen));
      break;
    case PLL_LOCK:
      printf(" PLL lock %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case PLL_ENABLE:
      printf(" PLL enable %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case PLL_SQUARE:
      printf(" PLL square %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case PLL_PHASE:
      printf(" PLL phase %g deg;",(180/M_PI)*decode_float(cp,optlen));
      break;
    case OUTPUT_CHANNELS:
      printf(" out channels %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case CALIBRATE:
      printf(" calibration %lg;",decode_double(cp,optlen));
      break;
    case AGC_ENABLE:
      printf(" agc enab %'llu;",(long long unsigned)decode_int(cp,optlen));
      break;
    case HEADROOM:
      printf(" headroom %lg dB;",decode_float(cp,optlen));
      break;
    case AGC_HANGTIME:
      printf(" hangtime %lg s;",decode_float(cp,optlen));
      break;
    case AGC_RECOVERY_RATE:
      printf(" recovery rate %lg dB/s;",decode_float(cp,optlen));
      break;
    case AGC_ATTACK_RATE:
      printf(" attack rate %lg dB/s;",decode_float(cp,optlen));
      break;
    case DESCRIPTION:
      printf(" %s;",decode_string(cp,optlen,sbuf,sizeof(sbuf)));
      break;
    case DIRECT_CONVERSION:
      printf(" direct conv %d;",(int)decode_int(cp,optlen));
      break;
    default:
      break;
    }
    cp += optlen;
  }
 done:;
  printf("\n");
}
