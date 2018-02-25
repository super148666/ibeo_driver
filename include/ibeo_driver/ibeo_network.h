/* Copyright 2013 Thomas Drage
 * File:   Network.h
 * Author: Thomas Drage to replace a library by Tim Black which I didn't have access to.
 * Credit: Wikipedia page on Berkeley Sockets API...
 *
 * Created on 3 Mar 2013.
 */

#ifndef CONTROL_SRC_IBEONETWORK_H_
#define CONTROL_SRC_IBEONETWORK_H_

class IBEONetwork {
 public:
  int socketFD;

  IBEONetwork();
      IBEONetwork(const IBEONetwork& orig);
      virtual ~IBEONetwork();

  bool IsConnected();
  void Disconnect();
  bool Connect(char * ip_addr, int port);
};

#endif  // CONTROL_SRC_IBEONETWORK_H_
