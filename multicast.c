#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include "multicast.h"

static void soptions(int fd){
  // Failures here are not fatal
  int reuse = 1;
  if(setsockopt(fd,SOL_SOCKET,SO_REUSEPORT,&reuse,sizeof(reuse)) != 0)
    perror("so_reuseport failed");
  if(setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse)) != 0)
    perror("so_reuseaddr failed");
  struct linger linger;
  linger.l_onoff = 0;
  linger.l_linger = 0;
  if(setsockopt(fd,SOL_SOCKET,SO_LINGER,&linger,sizeof(linger)) != 0)
    perror("so_linger failed");
}

#ifdef __linux__ // Linux, etc, for both IPv4/IPv6
static int join_group(int fd,struct addrinfo *resp){
  struct sockaddr_in const *sin = (struct sockaddr_in *)resp->ai_addr;
  if(!IN_MULTICAST(sin->sin_addr.s_addr))
    return -1;

  struct group_req group_req;
  group_req.gr_interface = 0;
  memcpy(&group_req.gr_group,resp->ai_addr,resp->ai_addrlen);
  if(setsockopt(fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0){
    perror("multicast join");
    return -1;
  }
  return 0;
}
#else // old version, seems required on Apple    
static int join_group(int fd,struct addrinfo *resp){
  struct sockaddr_in const *sin = (struct sockaddr_in *)resp->ai_addr;
  if(!IN_MULTICAST(sin->sin_addr.s_addr))
    return -1;

  struct ip_mreq mreq;
  //  mreq.imr_multiaddr.s_addr = sin->sin_addr.s_addr;
  mreq.imr_multiaddr = sin->sin_addr;
  mreq.imr_interface.s_addr = INADDR_ANY;
  if(setsockopt(fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq)) != 0){
    perror("multicast join");
    return -1;
  }
  return 0;
}
#endif

// Set up multicast socket for input or output
int setup_mcast(char const *addr,char const *port,int output){
  struct addrinfo hints;
  memset(&hints,0,sizeof(hints));
  hints.ai_family = AF_INET; // Only IPv4 for now (grrr....)
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_NUMERICSERV;

  struct addrinfo *results = NULL;
  int ecode;
  if((ecode = getaddrinfo(addr,port,&hints,&results)) != 0){
    fprintf(stderr,"setup_input getaddrinfo(%s,%s): %s\n",addr,port,gai_strerror(ecode));
    return -1;
  }
  struct addrinfo *resp;
  int fd = -1;
  for(resp = results; resp != NULL; resp = resp->ai_next){
    if((fd = socket(resp->ai_family,resp->ai_socktype,resp->ai_protocol)) < 0)
      continue;
    soptions(fd);
    if(output){
      if((connect(fd,resp->ai_addr,resp->ai_addrlen) == 0))
	break;
    } else { // input
      if((bind(fd,resp->ai_addr,resp->ai_addrlen) == 0))
	break;
    }
    close(fd);
    fd = -1;
  }
  // Strictly speaking, it is not necessary to join a multicast group to which we only send.
  // But this creates a problem with brain-dead Netgear (and probably other) "smart" switches
  // that do IGMP snooping. There's a setting to handle what happens with multicast groups
  // to which no IGMP messages are seen. If set to discard them, IPv6 multicast breaks
  // because there's no IPv6 multicast querier. But set to pass them, then IPv4 multicasts
  // that aren't subscribed to by anybody are flooded everywhere! We avoid that by subscribing
  // to our own multicasts.

  if(fd != -1)
    join_group(fd,resp);
  else
    fprintf(stderr,"setup_input: Can't create input multicast socket from %s:%s\n",addr,port);

  freeaddrinfo(results);
  return fd;
}
