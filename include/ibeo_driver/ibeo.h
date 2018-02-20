/* Copyright 2012 Timothy Black
 * File:   IBEO.h
 * Author: Timothy Black (20373081)
 *
 * Created on 18 April 2012, 7:34 PM
 * Modified, T. Drage, 2013.
 * Modified, T. Churack, 2015.
 */

#ifndef CONTROL_SRC_IBEO_H_
#define CONTROL_SRC_IBEO_H_

#include <inttypes.h>
#include <stdlib.h>
#include <deque>


#define IBEO_IP_ADDRESS "192.168.2.4"  // Default IP Address for the ibeo.
#define IBEO_PORT 12002                // Default Port for the ibeo.

#define MSG_BUFFERS 2  // Data buffers for data storage.

#define MAX_SCAN_POINTS 65536   // Maximum scan points.
#define MAX_OBJECTS 20          // Maximum objects stored.
#define MAX_CONTOUR_POINTS 100  // Maximum contour points per scan.

#define IBEO_PERIOD 0.4  // Minimum period between analysing IBEO data.

#define OBJECT_THRESHOLD \
  15  // Distance outside which all points are considered road.

#define MAXARRAYSIZE 500

#define LAYERS 4

namespace lidar {
class IBEONetwork;

/**
 * Purpose: ibeo header structure.
 *
 * Notes:   excludes the 4-byte magic word found at beginning of header.
 */
struct IBEO_HEADER {
  unsigned int size_prev_msg;
  unsigned int size_cur_msg;
  char res;
  char dev_id;
  unsigned short data_type;
  uint64_t ntp_time;
};

/**
 * Purpose: data header for each scan.
 */
struct SCAN_DATA_HEADER {
  unsigned short scan_number;
  unsigned short scanner_status;
  unsigned short sync_phase_offset;
  uint64_t scan_start_time;
  uint64_t scan_stop_time;
  unsigned short angle_ticks_per_rotation;
  short start_angle;
  short stop_angle;
  unsigned short scan_points;
  short mp_yaw;
  short mp_pitch;
  short mp_roll;
  short mp_x;
  short mp_y;
  short mp_z;
  unsigned short flags;
};

/**
 * Purpose: structure to hold data for each scan point.
 */

struct SCAN_DATA_POINT {
  char layer_echo;
  char flags;
  short horiz_angle;
  unsigned short radial_dist;
  unsigned short echo_pulse_width;
  unsigned short res;
};

struct SCAN_XY_DATA {
  std::deque<double> xvalues, yvalues, zvalues;
};

/**
 * Purpose: structure to hold an x, y coordinate for a scan point.
 */
struct POINT_2D {
  short x;
  short y;
};

/**
 * Purpose: structure to hold an x, y coordinate for a size.
 */
struct SIZE_2D {
  unsigned short x;
  unsigned short y;
};

/**
 * Purpose: header for objects found by the ibeo.
 */
struct OBJECT_DATA_HEADER {
  uint64_t scan_start_time;
  unsigned short number_of_objects;
};

/**
 * Purpose: data for each object found by the ibeo.
 */
struct OBJECT_DATA {
  unsigned short object_id;
  unsigned short object_age;
  unsigned short object_prediction_age;
  unsigned short relative_timestamp;
  POINT_2D reference_point;
  POINT_2D reference_point_sigma;
  POINT_2D closest_point;
  POINT_2D bounding_box_center;
  unsigned short bounding_box_width;
  unsigned short bounding_box_length;
  POINT_2D object_box_center;
  SIZE_2D object_box_size;
  short object_box_orientation;
  POINT_2D absolute_velocity;
  SIZE_2D absolute_velocity_sigma;
  POINT_2D relative_velocity;
  unsigned short classification;
  unsigned short classification_age;
  unsigned short classification_certainty;
  unsigned short number_contour_points;
  POINT_2D contour_points[MAX_CONTOUR_POINTS];
};

/**
 * Purpose: holds the data relating to errors experienced by the ibeo.
 */
struct ERROR_DATA {
  unsigned short error_register_1;
  unsigned short error_register_2;
  unsigned short warning_register_1;
  unsigned short warning_register_2;
  char res[8];
};

class IBEO {
 public:
  int curScanDataSource;
  int curObjectDataSource;
  int curErrorDataSource;
  bool inUse;

  int layersToScan;

  SCAN_DATA_HEADER scan_data_header[MSG_BUFFERS];
  SCAN_DATA_POINT scan_data_points[MSG_BUFFERS][MAX_SCAN_POINTS];
  OBJECT_DATA_HEADER object_data_header[MSG_BUFFERS];
  OBJECT_DATA object_data[MSG_BUFFERS][MAX_OBJECTS];
  ERROR_DATA error_data[MSG_BUFFERS];
  SCAN_XY_DATA current_xy_scan;

  IBEO();
  IBEO(const IBEO& orig);
  virtual ~IBEO();
  bool Open(char*, int);
  bool Open();
  void Start();
  void StopScanner();
  void ReadMessages();
  IBEO_HEADER FindHeader();

 private:
  bool Run;

  boost::thread m_Thread;
  timeval lastwrite;

  IBEONetwork* connection;

  template <class T>
  inline bool Recv(T);

  bool Read_Scan_Data();
  bool Read_Object_Data();
  bool Read_Errors();
  bool Read_Point2D(POINT_2D*);
  bool Read_Size2D(SIZE_2D*);

  void ProcessMessages();
};

}  // namespace lidar

#endif  // CONTROL_SRC_IBEO_H_
