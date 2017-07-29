#ifndef _MULTICAST_H
#define _MULTICAST_H
int setup_mcast_input(char const *addr,char const *port);
int setup_mcast_output(char const *addr,char const *port);
extern char Mcast_dest_port[];
#endif
