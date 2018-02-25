/* Copyright 2013 Thomas Drage
 * File:   IBEONetwork.cpp
 * Author: Thomas Drage to replace a library by Tim Black which I didn't have access to.
 * Credit: Wikipedia page on Berkeley Sockets API...
 *
 * Created on 3 Mar 2013.
 */


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <iostream>

#include "ibeo_driver/ibeo_network.h"

/**
 * Purpose: Creates a new instance of the Network object.
 * Inputs : None.
 * Outputs: None.
 */
IBEONetwork::IBEONetwork() {

  socketFD = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

  if (-1 == socketFD) {
    perror("cannot create socket");
    exit(EXIT_FAILURE);
  }
}

/**
 * Purpose: Creates a new instance of the Network object.
 * Inputs : An Network object.
 * Outputs: None.
 */
IBEONetwork::IBEONetwork(const IBEONetwork& orig) {
}

/**
 * Purpose: Destroys the instance of the Network object.
 * Inputs : None.
 * Outputs: None.
 */
IBEONetwork::~IBEONetwork() {
  // if (connection->IsConnected()) connection->Disconnect();
}

bool IBEONetwork::Connect(char * ip_addr, int port) {

  struct sockaddr_in stSockAddr;
  int Res;

  memset(&stSockAddr, 0, sizeof(stSockAddr));

  stSockAddr.sin_family = AF_INET;
  stSockAddr.sin_port = htons(port);
  Res = inet_pton(AF_INET, ip_addr, &stSockAddr.sin_addr);

  if (0 > Res) {
    perror("error: first parameter is not a valid address family");
    close(socketFD);
    return false;
  } else if (0 == Res) {
    perror("char string (second parameter does not contain valid ipaddress)");
    close(socketFD);
    return false;
  }

  if (-1 == connect(socketFD, (struct sockaddr *)&stSockAddr, sizeof(stSockAddr))) {
    perror("connect failed");
    close(socketFD);
    return false;
  }

  return true;
}

bool IBEONetwork::IsConnected() {

  int act;
  char buff;

  act = recv(socketFD, &buff, 1, MSG_PEEK);

  if (act) {
    return true;
  } else {
    return false;
  }
}

void IBEONetwork::Disconnect() {

  (void) shutdown(socketFD, SHUT_RDWR);

  close(socketFD);

}
