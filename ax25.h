// $Id: ax25.h,v 1.2 2018/04/22 22:32:59 karn Exp karn $
// Functions defined in ax25.c
// Copyright 2018, Phil Karn, KA9Q

// AX.25 frame, broken down
#define MAX_DIGI 10
#define CALL_LEN 16 // Actually could be 9
#define MAX_INFO 256

struct digi {
  char name[CALL_LEN];
  int h; // Has been repeated bit
};

struct ax25_frame {
  char dest[CALL_LEN];  // printable representations
  char source[CALL_LEN];
  struct digi digipeaters[MAX_DIGI];
  int ndigi;
  int control;
  int type;
  char information[MAX_INFO];
  int info_len;
};


int ax25_parse(struct ax25_frame *out,unsigned char *in,int len);
int dump_frame(unsigned char *frame,int bytes);
int crc_good(unsigned char *frame,int length);
char *get_callsign(char *result,unsigned char *in);
int decode_base91(char *in);

