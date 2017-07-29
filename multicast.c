#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include "multicast.h"

// Set up input socket for multicast data stream from front end
int setup_mcast_input(char const *addr,char const *port){
  int fd = -1;
  struct addrinfo hints,*results,*resp;

  memset(&hints,0,sizeof(hints));
  hints.ai_family = AF_INET; // Only IPv4 for now (grrr....)
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_NUMERICSERV;

  int ecode;
  if((ecode = getaddrinfo(addr,port,&hints,&results)) != 0){
    fprintf(stderr,"setup_input getaddrinfo(%s,%s): %s\n",addr,port,gai_strerror(ecode));
    return -1;
  }
  for(resp = results; resp != NULL; resp = resp->ai_next){
    if((fd = socket(resp->ai_family,resp->ai_socktype,resp->ai_protocol)) < 0)
      continue;
    
    // Failures here are not fatal
    int reuse = 1;
    if(setsockopt(fd,SOL_SOCKET,SO_REUSEPORT,&reuse,sizeof(reuse)) != 0)
      perror("so_reuseport failed");
    if(setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse)) != 0)
      perror("so_reuseaddr failed");
    
    if((bind(fd,resp->ai_addr,resp->ai_addrlen) == 0))
      break;
    close(fd);
    fd = -1;
  }
  freeaddrinfo(results);
  if(fd == -1){
    fprintf(stderr,"setup_input: Can't create input multicast socket from %s:%s\n",addr,port);
    return -1;
  }
  
#ifdef __linux__ // Linux, etc, for both IPv4/IPv6
  struct group_req group_req;
  group_req.gr_interface = 0;
  memcpy(&group_req.gr_group,resp->ai_addr,resp->ai_addrlen);
  if(setsockopt(fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0)
    perror("ip multicast join");

#else // old version, seems required on Apple    
  struct sockaddr_in const *sin = (struct sockaddr_in *)resp->ai_addr;
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = sin->sin_addr.s_addr;
  mreq.imr_interface.s_addr = INADDR_ANY;
  if(setsockopt(fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq)) != 0)
    perror("ipv4 multicast join");
#endif
  return fd;
}
// Set up multicast output
int setup_mcast_output(char const *addr,char const *port){
  int fd = -1;
  struct addrinfo hints,*results,*resp;

  memset(&hints,0,sizeof(hints));
  hints.ai_family = AF_INET; // Only IPv4 for now (grrr....)
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_NUMERICSERV;

  int ecode;
  if((ecode = getaddrinfo(addr,port,&hints,&results)) != 0){
    fprintf(stderr,"setup_output getaddrinfo(%s,%s): %s\n",addr,port,gai_strerror(ecode));
    return -1;
  }
  for(resp = results; resp != NULL; resp = resp->ai_next){
    if((fd = socket(resp->ai_family,resp->ai_socktype,resp->ai_protocol)) < 0)
      continue;
    if((connect(fd,resp->ai_addr,resp->ai_addrlen) == 0))
      break;
    close(fd);
    fd = -1;
  }
  freeaddrinfo(results);  
  if(fd == -1){
    fprintf(stderr,"setup_output: Can't create multicast socket to %s:%s\n",addr,port);
    return -1;
  }

  // Failures here are not fatal
  int reuse = 1;
  if(setsockopt(fd,SOL_SOCKET,SO_REUSEPORT,&reuse,sizeof(reuse)) != 0)
    perror("so_reuseport failed");
  if(setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse)) != 0)
    perror("so_reuseaddr failed");

  // Strictly speaking, it is not necessary to join a multicast group to which we only send.
  // But this creates a problem with brain-dead Netgear (and probably other) "smart" switches
  // that do IGMP snooping. There's a setting to handle what happens with multicast groups
  // to which no IGMP messages are seen. If set to discard them, IPv6 multicast breaks
  // because there's no IPv6 multicast querier. But set to pass them, then IPv4 multicasts
  // that aren't subscribed to by anybody are flooded everywhere! We avoid that by subscribing
  // to our own multicasts.
  
#ifdef __linux__ // Linux, etc, for both IPv4/IPv6
  struct group_req group_req;
  group_req.gr_interface = 0;
  memcpy(&group_req.gr_group,resp->ai_addr,resp->ai_addrlen);
  if(setsockopt(fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0)
    perror("ip multicast join");
#else // old version, seems required on Apple    
  struct sockaddr_in const *sin = (struct sockaddr_in *)resp->ai_addr;
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = sin->sin_addr.s_addr;
  mreq.imr_interface.s_addr = INADDR_ANY;
  if(setsockopt(fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq)) != 0)
    perror("ipv4 multicast join");
#endif
  return fd;
}
