// $Id$
// Routines for reading and writing text strings to external file attributes
// Should be portable to Linux and Mac OSX, which are gratuitously different
int attrscanf(int fd,char const *name,char const *format, ...);
int attrprintf(int fd,char const *attr,char const *format, ...);
