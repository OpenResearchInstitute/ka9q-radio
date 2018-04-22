// $Id: attr.h,v 1.1 2017/07/29 10:17:45 karn Exp karn $
// Routines for reading and writing formatted text strings to external file attributes
// Should be portable to Linux and Mac OSX, which are gratuitously different
// Copyright 2018, Phil Karn, KA9Q

int attrscanf(int fd,char const *name,char const *format, ...);
int attrprintf(int fd,char const *attr,char const *format, ...);
