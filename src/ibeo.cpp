/* Copyright 2013 Thomas Drage
 * File:   IBEO.cpp
 * Author: Timothy Black (20373081), modifications Thomas Drage (20510505).
 *
 * Created on 18 April 2012, 7:34 PM
 * Modified 3 March 2013, 23 July 2013 (T.Drage).
 *
 * Road finding and object projection T. Drage Sep 2013.
 */

#include "ibeo.h"

#include <stdio.h>

#include <arpa/inet.h>
#include <sys/socket.h>

#include <boost/thread.hpp>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

#include "lidar/ibeo_network.h"
#include "ros/ros.h"


#define SOCKET_FD connection->socketFD

namespace lidar {

static char ibeo_magic_word[] = {
    static_cast<char>(0xAF), static_cast<char>(0xFE), static_cast<char>(0xC0),
    static_cast<char>(
        0xC2)}; // The magic word to look for for beginning of message.

int curScanDataSource = 0;   // Current scan data source/buffer.
int curObjectDataSource = 0; // Current object data source/buffer.
int curErrorDataSource = 0;  // Current error data source/buffer.

bool gotObject = false;
bool gotScan = false;

bool verbose = false;

char byteIn = 0;

/**
 * Purpose: Creates a new instance of the IBEO object.
 * Inputs : None.
 * Outputs: None.
 */
IBEO::IBEO() {
  gettimeofday(&lastwrite, NULL);
  layersToScan = LAYERS;

  connection = new IBEONetwork();

  for (int i = 0; i < MSG_BUFFERS; i++) {
    scan_data_header[i].scan_points = 0;
    object_data_header[i].number_of_objects = 0;
  }
  inUse = false;
}

/**
 * Purpose: Creates a new instance of the IBEO object.
 * Inputs : An IBEO object.
 * Outputs: None.
 */
IBEO::IBEO(const IBEO &orig) {}

/**
 * Purpose: Destroys the instance of the IBEO object.
 * Inputs : None.
 * Outputs: None.
 */
IBEO::~IBEO() {
  if (connection->IsConnected())
    connection->Disconnect();
  delete connection;
}

/**
 * Purpose: Starts the communication between device and ibeo scanner.
 * Inputs : IP Address and port to connect to.
 * Outputs: true if successful, false otherwise.
 */
bool IBEO::Open(char *ip_addr, int port) {
  Log->WriteLogLine("IBEO scanner - attempting to connect to " +
                    boost::lexical_cast<std::string>(ip_addr) + ":" +
                    boost::lexical_cast<std::string>(port) + ".");
  if (!connection->Connect(ip_addr, port)) {
    Log->WriteLogLine("IBEO scanner - Connecting failed...");
    return false;
  }
  inUse = true;
  Log->WriteLogLine("IBEO scanner - connected to scanner successfully.");

  return true;
}

/**
 * Purpose: Connects to the ibeo scanner using the default IP Address and port.
 * Inputs : None.
 * Outputs: true if successful, false otherwise.
 */
bool IBEO::Open() { return Open((char *)IBEO_IP_ADDRESS, IBEO_PORT); }

/**
 * Purpose: Disconnects from the ibeo scanner.
 * Inputs : None.
 * Outputs: None.
 */
void IBEO::StopScanner() {
  connection->Disconnect();
  inUse = false;
  Run = false;
  sleep(1);
  if (verbose) {
    std::cout << "ibeo scanner: disconnected from scanner." << std::endl;
  }
}

/**
 * Purpose: Finds the ibeo header from the data buffer.
 * Inputs : None.
 * Outputs: The IBEO_HEADER object.
 */
IBEO_HEADER IBEO::FindHeader() {
  bool foundHeader;
  IBEO_HEADER headerIn;
  byteIn = 0;
  int i;
  char *pointByte = &byteIn;
  foundHeader = false;

  while (!foundHeader) {
    for (i = 0; i < 4; i++) {
      try {
        if (recv(SOCKET_FD, pointByte, 1, 0) > 0) {
          if (byteIn != ibeo_magic_word[i])
            break;
        } else {
          throw 1;
        }
      } catch (int e) {
        std::cout << "Caught 1" << std::endl;
      }
    }
    if (i == 4)
      foundHeader = true;
  }

  if (recv(SOCKET_FD, &headerIn.size_prev_msg, sizeof(headerIn.size_prev_msg),
           0) != sizeof(headerIn.size_prev_msg))
    throw 1;
  if (recv(SOCKET_FD, &headerIn.size_cur_msg, sizeof(headerIn.size_cur_msg),
           0) != sizeof(headerIn.size_cur_msg))
    throw 1;
  if (recv(SOCKET_FD, &headerIn.res, sizeof(headerIn.res), 0) !=
      sizeof(headerIn.res))
    throw 1;
  if (recv(SOCKET_FD, &headerIn.dev_id, sizeof(headerIn.dev_id), 0) !=
      sizeof(headerIn.dev_id))
    throw 1;
  if (recv(SOCKET_FD, &headerIn.data_type, sizeof(headerIn.data_type), 0) !=
      sizeof(headerIn.data_type))
    throw 1;
  if (recv(SOCKET_FD, &headerIn.ntp_time, sizeof(headerIn.ntp_time), 0) !=
      sizeof(headerIn.ntp_time))
    throw 1;

  if (verbose) {
    std::cout << "ibeo scanner: found header for device id "
              << htons(headerIn.dev_id) << "." << std::endl;
    std::cout << "              data type for next message is "
              << htons(headerIn.data_type) << "." << std::endl;
    std::cout << "              size of next message is "
              << htons(headerIn.size_cur_msg) << "." << std::endl;
  }
  return headerIn;
}

/**
 * Purpose: Reads messages from the ibeo.
 * Inputs : None.
 * Outputs: None.
 */
void IBEO::ReadMessages() {
  IBEO_HEADER header;

  gotObject = false;
  gotScan = false;

  while (!(gotObject == true &&
           gotScan == true)) { // Scan until we have one of each set of data.
    try {
      header = this->FindHeader();
      switch (htons(header.data_type)) {
      case 0x2202:
        if (verbose)
			std::cout << "ibeo scanner: found scan data." << std::endl;
        if (!Read_Scan_Data())
          return;
        break;
      case 0x2221:
        if (verbose)
          std::cout << "ibeo scanner: found object data." << std::endl;
        if (!Read_Object_Data())
          return;
        break;
      case 0x2030:
        if (verbose)
          std::cout << "ibeo scanner: found error data." << std::endl;
        if (!Read_Errors())
          return;
        break;
      default:
        if (verbose)
          std::cout << 
              "ibeo scanner: received unknown message with id " << std::endl; //+
        // htons(header.data_type));
      }

    } catch (int e) {
      std::cout << "IBEO - caught error!" << std::endl;
      return;
    }
  }

  read(SOCKET_FD, NULL, 1000000);
}

/**
 * Purpose: Reads the scan data from the ibeo scanner.
 * Inputs : None.
 * Outputs: true if successful, false otherwise.
 */

#define SCAN_HEADER scan_data_header[next_scan_source]
#define SCAN_POINTS scan_data_points[next_scan_source][i]

bool IBEO::Read_Scan_Data() {
  int next_scan_source;
  int i;

  next_scan_source = (this->curScanDataSource + 1) % 2;
  if (Recv(SCAN_HEADER.scan_number))
    return false;
  if (Recv(SCAN_HEADER.sync_phase_offset))
    return false;
  if (Recv(SCAN_HEADER.scan_start_time))
    return false;
  if (Recv(SCAN_HEADER.scan_stop_time))
    return false;
  if (Recv(SCAN_HEADER.angle_ticks_per_rotation))
    return false;
  if (Recv(SCAN_HEADER.start_angle))
    return false;
  if (Recv(SCAN_HEADER.stop_angle))
    return false;
  if (Recv(SCAN_HEADER.scan_points))
    return false;
  if (Recv(SCAN_HEADER.mp_yaw))
    return false;
  if (Recv(SCAN_HEADER.mp_pitch))
    return false;
  if (Recv(SCAN_HEADER.mp_roll))
    return false;
  if (Recv(SCAN_HEADER.mp_x))
    return false;
  if (Recv(SCAN_HEADER.mp_y))
    return false;
  if (Recv(SCAN_HEADER.mp_z))
    return false;
  if (Recv(SCAN_HEADER.flags))
    return false;

  if (verbose)
    std::cout << "ibeo scanner: reading " << SCAN_HEADER.scan_points
              << " points from ibeo." << std::endl;
  if (verbose)
    std::cout << "ibeo scanner: start angle " << SCAN_HEADER.start_angle
              << " degrees." << std::endl;
  if (verbose)
    std::cout << "ibeo scanner: stop angle " << SCAN_HEADER.stop_angle
              << " degrees." << std::endl;

  for (i = 0; i < SCAN_HEADER.scan_points; i++) {
    if (Recv(SCAN_POINTS.layer_echo))
      return false;
    if (Recv(SCAN_POINTS.flags))
      return false;
    if (Recv(SCAN_POINTS.horiz_angle))
      return false;
    if (Recv(SCAN_POINTS.radial_dist))
      return false;
    if (Recv(SCAN_POINTS.echo_pulse_width))
      return false;
    if (Recv(SCAN_POINTS.res))
      return false;
  }

  curScanDataSource = next_scan_source;
  gotScan = true;
  return true;
}

/**
 * Purpose: Reads object data from the ibeo scanner.
 * Inputs : None.
 * Outputs: true if successful, false otherwise.
 */

#define OBJ_HEADER object_data_header[next_object_source]
#define OBJ_DATA object_data[next_object_source][i]

bool IBEO::Read_Object_Data() {
  int next_object_source;
  int i, j;

  next_object_source = (this->curObjectDataSource + 1) % 2;
  if (Recv(OBJ_HEADER.scan_start_time))
    return false;
  if (Recv(OBJ_HEADER.number_of_objects))
    return false;

  if (verbose)
    std::cout << "ibeo scanner: reading " << OBJ_HEADER.number_of_objects
              << " objects from ibeo." << std::endl;

  if (OBJ_HEADER.number_of_objects > MAX_OBJECTS)
    OBJ_HEADER.number_of_objects = MAX_OBJECTS;

  for (i = 0; i < OBJ_HEADER.number_of_objects; i++) {
    if (Recv(OBJ_DATA.object_id))
      return false;
    if (Recv(OBJ_DATA.object_age))
      return false;
    if (Recv(OBJ_DATA.object_prediction_age))
      return false;
    if (Recv(OBJ_DATA.relative_timestamp))
      return false;

    if (!Read_Point2D(&OBJ_DATA.reference_point))
      return false;
    if (!Read_Point2D(&OBJ_DATA.reference_point_sigma))
      return false;
    if (!Read_Point2D(&OBJ_DATA.closest_point))
      return false;
    if (!Read_Point2D(&OBJ_DATA.bounding_box_center))
      return false;

    if (Recv(OBJ_DATA.bounding_box_width))
      return false;
    if (Recv(OBJ_DATA.bounding_box_length))
      return false;

    if (!Read_Point2D(&OBJ_DATA.object_box_center))
      return false;
    if (!Read_Size2D(&OBJ_DATA.object_box_size))
      return false;

    if (Recv(OBJ_DATA.object_box_orientation))
      return false;

    if (!Read_Point2D(&OBJ_DATA.absolute_velocity))
      return false;
    if (!Read_Size2D(&OBJ_DATA.absolute_velocity_sigma))
      return false;
    if (!Read_Point2D(&OBJ_DATA.relative_velocity))
      return false;

    if (Recv(OBJ_DATA.classification))
      return false;
    if (Recv(OBJ_DATA.classification_age))
      return false;
    if (Recv(OBJ_DATA.classification_certainty))
      return false;
    if (Recv(OBJ_DATA.number_contour_points))
      return false;

    if (OBJ_DATA.number_contour_points > MAX_CONTOUR_POINTS)
      OBJ_DATA.number_contour_points = MAX_CONTOUR_POINTS;
    for (j = 0; j < OBJ_DATA.number_contour_points; j++) {
      if (!Read_Point2D(&OBJ_DATA.contour_points[j]))
        return false;
    }
  }

  curObjectDataSource = next_object_source;
  gotObject = true;
  return true;
}

/**
 * Purpose: Reads the errors from the ibeo scanner.
 * Inputs : None.
 * Outputs: true if successful, false otherwise.
 */

#define ERRORS error_data[next_error_source]

bool IBEO::Read_Errors() {
  int next_error_source;

  if (verbose)
    std::cout << "ibeo scanner: receiving errors from the ibeo." << std::endl;

  next_error_source = (this->curErrorDataSource + 1) % 2;
  if (Recv(ERRORS.error_register_1))
    return false;
  if (Recv(ERRORS.error_register_2))
    return false;
  if (Recv(ERRORS.warning_register_1))
    return false;
  if (Recv(ERRORS.warning_register_2))
    return false;
  if (Recv(ERRORS.res))
    return false;

  return true;
}

/**
 * Purpose: Recieves data from the ibeo scanner.
 * Inputs : The data storage location.
 * Outputs: true if successful, false otherwise.
 */
template <class T> inline bool IBEO::Recv(T msg) {
  return recv(SOCKET_FD, &(msg), sizeof(msg), 0) != sizeof(msg);
}

/**
 * Purpose: Reads the POINT_2D from the ibeo scanner.
 * Inputs : The POINT_2D storage location.
 * Outputs: true if successful, false otherwise.
 */
bool IBEO::Read_Point2D(POINT_2D *pointIn) {
  if (recv(SOCKET_FD, &pointIn->x, sizeof(short), 0) != sizeof(short))
    return false;
  if (recv(SOCKET_FD, &pointIn->y, sizeof(short), 0) != sizeof(short))
    return false;
  return true;
}

/**
 * Purpose: Reads the SIZE_2D from the ibeo scanner.
 * Inputs : The SIZE_2D storage location.
 * Outputs: true if successful, false otherwise.
 */
bool IBEO::Read_Size2D(SIZE_2D *sizeIn) {
  if (recv(SOCKET_FD, &sizeIn->x, sizeof(short), 0) != sizeof(short))
    return false;
  if (recv(SOCKET_FD, &sizeIn->y, sizeof(short), 0) != sizeof(short))
    return false;
  return true;
}

/*
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * All code after this point was written by T. Drage, Sep 2013.
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */

/**
 * Purpose: Start the IBEO sensor processing incoming messages.
 * Inputs : None.
 * Outputs: None.
 */
void IBEO::Start() {
  if (inUse) {
    Run = true;

    m_Thread = boost::thread(&IBEO::ProcessMessages, this);
    m_Thread.detach();
  }
}

/**
 * Purpose: Contains the main loop that deals with IBEO data.
 * Inputs : None.
 * Outputs: None.
 */
void IBEO::ProcessMessages() {
  while (Run) {
    ReadMessages();

    timeval current;
    gettimeofday(&current, NULL);

    // Save CPU time by not acting upon every set of data.
    if ((current.tv_sec + ((double)current.tv_usec) / 1000000) >
        (lastwrite.tv_sec + ((double)lastwrite.tv_usec) / 1000000) +
            IBEO_PERIOD) {

      gettimeofday(&lastwrite, NULL);
      //      Log->WriteLogLine("After Get Time of Day...");
    }
  }
}


} // namespace lidar


void main(int argc, char** argv)
{
	ros::init(
}
