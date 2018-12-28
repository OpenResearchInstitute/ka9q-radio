// $Id: multicast.c,v 1.37 2018/12/09 12:12:20 karn Exp karn $
// Multicast socket and RTP utility routines
// Copyright 2018 Phil Karn, KA9Q

#include <stdio.h>
#include <unistd.h>
#include <netdb.h>
#include <string.h>
#include <net/if.h>
#if defined(linux)
#include <bsd/string.h>
#endif
#include "multicast.h"

#define EF_TOS 0x2e // Expedited Forwarding type of service, widely used for VoIP (which all this is, sort of)

// Set options on multicast socket
static void soptions(int fd,int mcast_ttl){
  // Failures here are not fatal
#if defined(linux)
  int freebind = 1;
  if(setsockopt(fd,IPPROTO_IP,IP_FREEBIND,&freebind,sizeof(freebind)) != 0)
    perror("freebind failed");
#endif

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

  u_char ttl = mcast_ttl;
  if(setsockopt(fd,IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof(ttl)) != 0)
    perror("so_ttl failed");

  u_char loop = 1;
  if(setsockopt(fd,IPPROTO_IP,IP_MULTICAST_LOOP,&loop,sizeof(loop)) != 0)
    perror("so_ttl failed");

  int tos = EF_TOS << 2; // EF (expedited forwarding)
  setsockopt(fd,IPPROTO_IP,IP_TOS,&tos,sizeof(tos));
}

// Join a socket to a multicast group
#if __APPLE__
// Workaround for joins on OSX (and BSD?) for default interface
// join_group works on apple only when interface explicitly specified
static int apple_join_group(int fd,struct sockaddr *sock){
  struct sockaddr_in *sin = (struct sockaddr_in *)sock;
  struct sockaddr_in6 *sin6 = (struct sockaddr_in6 *)sock;
  struct ip_mreq mreq;
  struct ipv6_mreq ipv6_mreq;

  if(fd < 0)
    return -1;
  switch(sin->sin_family){
  case PF_INET:
    if(!IN_MULTICAST(ntohl(sin->sin_addr.s_addr)))
      return -1;
    mreq.imr_multiaddr = sin->sin_addr;
    mreq.imr_interface.s_addr = INADDR_ANY; // Default interface
    if(setsockopt(fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq)) != 0){
      perror("multicast v4 join");
      return -1;
    }
    break;
  case PF_INET6:
    if(!IN6_IS_ADDR_MULTICAST(&sin6->sin6_addr))
      return -1;
    ipv6_mreq.ipv6mr_multiaddr = sin6->sin6_addr;
    ipv6_mreq.ipv6mr_interface = 0; // Default interface

    if(setsockopt(fd,IPPROTO_IP,IPV6_JOIN_GROUP,&ipv6_mreq,sizeof(ipv6_mreq)) != 0){
      perror("multicast v6 join");
      return -1;
    }
    break;
  default:
    return -1; // Unknown address family
  }
  return 0;
}
#endif

static int join_group(int fd,struct sockaddr *sock,char *iface){
  struct sockaddr_in *sin = (struct sockaddr_in *)sock;
  struct sockaddr_in6 *sin6 = (struct sockaddr_in6 *)sock;

  if(fd < 0)
    return -1;

  int socklen = 0;

  // Ensure it's a multicast address
  // Is this check really necessary?
  // Maybe the setsockopt would just fail cleanly if it's not
  switch(sin->sin_family){
  case PF_INET:
    socklen = sizeof(struct sockaddr_in);
    if(!IN_MULTICAST(ntohl(sin->sin_addr.s_addr)))
      return -1;
    break;
  case PF_INET6:
    socklen = sizeof(struct sockaddr_in6);
    if(!IN6_IS_ADDR_MULTICAST(&sin6->sin6_addr))
      return -1;
    break;
  default:
    return -1; // Unknown address family
  }
#if __APPLE__
  if(!iface)
    return apple_join_group(fd,sock); // Apple workaround for default interface
#endif  

  struct group_req group_req;
  if(iface)
    group_req.gr_interface = if_nametoindex(iface);
  else
    group_req.gr_interface = 0; // Default interface    

  memcpy(&group_req.gr_group,sock,socklen);
  if(setsockopt(fd,IPPROTO_IP,MCAST_JOIN_GROUP,&group_req,sizeof(group_req)) != 0){
    perror("multicast join");
    return -1;
  }
  return 0;
}

// This is a bit messy. Is there a better way?
char Default_mcast_port[] = "5004";
char Default_rtcp_port[] = "5005";

// Set up multicast socket for input or output

// Target is in the form of domain.name.com:5004 or 1.2.3.4:5004
// when output = 1, connect to the multicast address so we can simply send() to it without specifying a destination
// when output = 0, bind to it so we'll accept incoming packets
// Add parameter 'offset' (normally 0) to port number; this will be 1 when sending RTCP messages
// (Can we just do both?)
int setup_mcast(char const *target,struct sockaddr *sock,int output,int ttl,int offset){
  int fd = -1;
  char *iface = NULL;

  struct sockaddr_storage sbuf;
  memset(&sbuf,0,sizeof(sbuf));

  if(target){
    if(sock == NULL)
      sock = (struct sockaddr *)&sbuf; // Use local temporary

    // Target specified, resolve it
    int len = strlen(target) + 1;  // Including terminal null
    char host[len],*port;
    
    strlcpy(host,target,len);
    if((iface = strrchr(host,',')) != NULL){
      // Interface specified
      *iface++ = '\0';
    }
    if((port = strrchr(host,':')) != NULL){
      *port++ = '\0';
    } else {
      port = Default_mcast_port; // Default for RTP
    }
    
    struct addrinfo hints;
    memset(&hints,0,sizeof(hints));
    hints.ai_family = PF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    hints.ai_flags = AI_ADDRCONFIG | AI_NUMERICSERV | (!output ? AI_PASSIVE : 0);
    
    struct addrinfo *results = NULL;
    int ecode = getaddrinfo(host,port,&hints,&results);
    if(ecode != 0){
      fprintf(stderr,"setup_mcast getaddrinfo(%s,%s): %s\n",host,port,gai_strerror(ecode));
      return -1;
    }
    // Use first entry on list -- much simpler
    // I previously tried each entry in turn until one succeeded, but with UDP sockets and
    // flags set to only return supported addresses, how could any of them fail?
    memcpy(sock,results->ai_addr,results->ai_addrlen);
    freeaddrinfo(results); results = NULL;

    switch(sock->sa_family){
    case AF_INET:
      {
	struct sockaddr_in *sinp = (struct sockaddr_in *)sock;
	sinp->sin_port = htons(ntohs(sinp->sin_port) + offset);
      }
      break;
    case AF_INET6:
      {
	struct sockaddr_in6 *sinp6 = (struct sockaddr_in6 *)sock;
	sinp6->sin6_port = htons(ntohs(sinp6->sin6_port) + offset);
      }
      break;
    }
  }
  if(sock == NULL)
    return -1; // Neither target or sock specified
  if((fd = socket(sock->sa_family,SOCK_DGRAM,0)) == -1)
    return -1;
      
  soptions(fd,ttl);
  if(output){
    if(connect(fd,sock,sizeof(struct sockaddr)) != 0){
      close(fd);
      return -1;
    }
  } else { // input
    if((bind(fd,sock,sizeof(struct sockaddr)) != 0)){
      close(fd);
      return -1;
    }
  }
  // Strictly speaking, it is not necessary to join a multicast group to which we only send.
  // But this creates a problem with brain-dead Netgear (and probably other) "smart" switches
  // that do IGMP snooping. There's a setting to handle what happens with multicast groups
  // to which no IGMP messages are seen. If set to discard them, IPv6 multicast breaks
  // because there's no IPv6 multicast querier. But set to pass them, then IPv4 multicasts
  // that aren't subscribed to by anybody are flooded everywhere! We avoid that by subscribing
  // to our own multicasts.
  join_group(fd,sock,iface);
  return fd;
}

// Convert RTP header from network (wire) big-endian format to internal host structure
// Written to be insensitive to host byte order and C structure layout and padding
// Use of unsigned formats is important to avoid unwanted sign extension
void  *ntoh_rtp(struct rtp_header *rtp,void  *data){
  uint32_t *dp = data;

  uint32_t w = ntohl(*dp++);
  rtp->version = w >> 30;
  rtp->pad = (w >> 29) & 1;
  rtp->extension = (w >> 28) & 1;
  rtp->cc = (w >> 24) & 0xf;
  rtp->marker = (w >> 23) & 1;
  rtp->type = (w >> 16) & 0x7f;
  rtp->seq = w & 0xffff;

  rtp->timestamp = ntohl(*dp++);
  rtp->ssrc = ntohl(*dp++);

  for(int i=0; i<rtp->cc; i++)
    rtp->csrc[i] = ntohl(*dp++);

  if(rtp->extension){
    int ext_len = ntohl(*dp++) & 0xffff;    // Ignore any extension, but skip over it
    dp += ext_len;
  }
  return dp;
}


// Convert RTP header from internal host structure to network (wire) big-endian format
// Written to be insensitive to host byte order and C structure layout and padding
void *hton_rtp(void *data, struct rtp_header *rtp){
  uint32_t *dp = data;

  *dp++ = htonl(RTP_VERS << 30 | rtp->pad << 29 | rtp->extension << 28 | (rtp->cc & 0xf) << 24 | rtp->marker << 23
		| (rtp->type & 0x7f) << 16 | rtp->seq);
  *dp++ = htonl(rtp->timestamp);
  *dp++ = htonl(rtp->ssrc);
  for(int i=0; i < rtp->cc; i++)
    *dp++ = htonl(rtp->csrc[i]);

  return dp;
}


// Process sequence number and timestamp in incoming RTP header:
// Check that the sequence number is (close to) what we expect
// If not, drop it but 3 wild sequence numbers in a row will assume a stream restart
//
// Determine timestamp jump, if any
// Returns: <0            if packet should be dropped as a duplicate or a wild sequence number
//           0            if packet is in sequence with no missing timestamps
//         timestamp jump if packet is in sequence or <10 sequence numbers ahead, with missing timestamps
int rtp_process(struct rtp_state *state,struct rtp_header *rtp,int sampcnt){
  if(rtp->ssrc != state->ssrc){
    // Normally this will happen only on the first packet in a session since
    // the caller demuxes the SSRC to multiple instances.
    // But a single-instance, interactive application like 'radio' lets the SSRC
    // change so it doesn't have to restart when the stream sender does.
    state->init = 0;
    state->ssrc = rtp->ssrc; // Must be filtered elsewhere if you want it
  }
  if(!state->init){
    state->packets = 0;
    state->seq = rtp->seq;
    state->timestamp = rtp->timestamp;
    state->dupes = 0;
    state->drops = 0;
    state->init = 1;
  }
  state->packets++;
  // Sequence number check
  short seq_step = (short)(rtp->seq - state->seq);
  if(seq_step != 0){
    if(seq_step < 0){
      state->dupes++;
      return -1;
    }
    state->drops += seq_step;
  }
  state->seq = rtp->seq + 1;

  int time_step = (int)(rtp->timestamp - state->timestamp);
  if(time_step < 0)
    return time_step;    // Old samples; drop. Shouldn't happen if sequence number isn't old

  state->timestamp = rtp->timestamp + sampcnt;
  return time_step;
}

// For caching back conversions of binary socket structures to printable addresses
void update_sockcache(struct sockcache *sc,struct sockaddr *sa){
  int len;
  switch(sa->sa_family){
  case AF_INET:
    len = sizeof(struct sockaddr_in);
    break;
  case AF_INET6:
    len = sizeof(struct sockaddr_in6);
    break;
  default: // shouldn't happen unless uninitialized
    len = 0;
    break;
  }
  if(memcmp(&sc->old_sockaddr,sa,len)){
    memcpy(&sc->old_sockaddr,sa,len);
    getnameinfo(sa,len,
		sc->host,NI_MAXHOST,
		sc->port,NI_MAXSERV,
		NI_NOFQDN|NI_NUMERICHOST|NI_NUMERICSERV);
  }
}

