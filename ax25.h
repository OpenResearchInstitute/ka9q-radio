// $Id$
// Functions defined in ax25.c
// Copyright 2018, Phil Karn, KA9Q

int dump_frame(unsigned char *frame,int bytes);
int crc_good(unsigned char *frame,int length);
char *get_callsign(char *result,unsigned char *in);
int decode_base91(char *in);

