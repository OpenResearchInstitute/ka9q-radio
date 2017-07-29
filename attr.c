// $Id$
// Read and write external file attributes
// These are in a separate file mainly because they are so OS-dependent. And gratuitously so.
// 29 July 2017 Phil Karn, KA9Q

#define _GNU_SOURCE 1
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <alloca.h>
#include <string.h>
#ifdef __linux__
#include <attr/xattr.h>
#else
#include <sys/xattr.h>
#endif

#include "attr.h"

// Look for external attribute "name" on an open file and perform scanf on its value
int attrscanf(int fd,char const *name,char const *format, ...){
  int r;


#ifdef __linux__   // Grrrrr.....
  char *temp = NULL;
  asprintf(&temp,"user.%s",name);
  r = fgetxattr(fd,temp,NULL,0); // How big is it?
  if(r < 0)
    return r;
  char *value = alloca(r);
  r = fgetxattr(fd,temp,value,r);
  free(temp);
#else // mainly OSX
  r = fgetxattr(fd,name,NULL,0,0,0); // How big is it?
  if(r < 0)
    return r;
  char *value = alloca(r);
  r = fgetxattr(fd,name,value,r,0,0);
#endif
  
  va_list ap;
  va_start(ap,format);
  r = vsscanf(value,format,ap);
  va_end(ap);
  
  return r;
}
int attrprintf(int fd,char const *attr,char const *format, ...){
  char *args = NULL;
  int r;
  
  va_list ap;
  va_start(ap,format);

  r = vasprintf(&args,format,ap);
  va_end(ap);
#ifdef __linux__  // Grrrrrrr....
  char *prefix;
  asprintf(&prefix,"user.%s",attr);
  if(fsetxattr(fd,prefix,args,strlen(args),0) == -1)
    r = -1;
  free(prefix);
#else
  if(fsetxattr(fd,attr,args,strlen(args),0,0) == -1)
    r = -1;
#endif

  free(args);
  return r;
}
