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

#include <stdio.h>

#include <arpa/inet.h>
#include <sys/socket.h>

#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <string>

#include "ibeo_driver/ibeo_network.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


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

    class IBEONetwork;

/**
 * Purpose: ibeo header structure.
 *
 * Notes:   excludes the 4-byte magic word found at beginning of header.
 */
    struct IBEO_HEADER {
        uint32_t size_prev_msg;
        uint32_t size_cur_msg;
        uint8_t res;
        uint8_t dev_id;
        uint16_t data_type;
        uint64_t ntp_time;
    };

/**
 * Purpose: data header for each scan.
 */
    struct SCAN_DATA_HEADER {
        uint16_t scan_number;
        uint16_t scanner_status;
        uint16_t sync_phase_offset;
        uint64_t scan_start_time;
        uint64_t scan_stop_time;
        uint16_t angle_ticks_per_rotation;
        int16_t start_angle;
        int16_t stop_angle;
        uint16_t scan_points;
        int16_t mp_yaw;
        int16_t mp_pitch;
        int16_t mp_roll;
        int16_t mp_x;
        int16_t mp_y;
        int16_t mp_z;
        uint16_t flags;
    };

/**
 * Purpose: structure to hold data for each scan point.
 */

    struct SCAN_DATA_POINT {
        uint8_t layer_echo;
        uint8_t flags;
        int16_t horiz_angle;
        uint16_t radial_dist;
        uint16_t echo_pulse_width;
        uint16_t res;
    };

    struct SCAN_XY_DATA {
        std::deque<double> xvalues, yvalues, zvalues;
    };

/**
 * Purpose: structure to hold an x, y coordinate for a scan point.
 */
    struct POINT_2D {
        int16_t x;
        int16_t y;
    };

/**
 * Purpose: structure to hold an x, y coordinate for a size.
 */
    struct SIZE_2D {
        uint16_t x;
        uint16_t y;
    };

/**
 * Purpose: header for objects found by the ibeo.
 */
    struct OBJECT_DATA_HEADER {
        uint64_t scan_start_time;
        uint16_t number_of_objects;
    };

/**
 * Purpose: data for each object found by the ibeo.
 */
    struct OBJECT_DATA {
        uint16_t object_id;
        uint16_t object_age;
        uint16_t object_prediction_age;
        uint16_t relative_timestamp;
        POINT_2D reference_point;
        POINT_2D reference_point_sigma;
        POINT_2D closest_point;
        POINT_2D bounding_box_center;
        uint16_t bounding_box_width;
        uint16_t bounding_box_length;
        POINT_2D object_box_center;
        SIZE_2D object_box_size;
        int16_t object_box_orientation;
        POINT_2D absolute_velocity;
        SIZE_2D absolute_velocity_sigma;
        POINT_2D relative_velocity;
        uint16_t classification;
        uint16_t classification_age;
        uint16_t classification_certainty;
        uint16_t number_contour_points;
        POINT_2D contour_points[MAX_CONTOUR_POINTS];
    };

/**
 * Purpose: holds the data relating to errors experienced by the ibeo.
 */
    struct ERROR_DATA {
        uint16_t error_register_1;
        uint16_t error_register_2;
        uint16_t warning_register_1;
        uint16_t warning_register_2;
        uint8_t res[8];
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

        IBEO(ros::NodeHandle nh);

        IBEO(const IBEO &orig);

        virtual ~IBEO();

        bool Open(char *, int);

        bool Open();
        float ConvertTicktsToAngle(int16_t angleTicks);
        void Start();

        void StopScanner();

        void ReadMessages();

        IBEO_HEADER FindHeader();

    private:
        bool Run;

        boost::thread m_Thread;
        timeval lastwrite;

        IBEONetwork *connection;
        ros::NodeHandle nh;
        sensor_msgs::LaserScan laserscan;
        ros::Publisher scan_pub;

        template<class T>
        inline bool Recv(T);

        bool Read_Scan_Data();

        bool Read_Object_Data();

        bool Read_Errors();

        bool Read_Point2D(POINT_2D *);

        bool Read_Size2D(SIZE_2D *);

        void ProcessMessages();
    };


#endif  // CONTROL_SRC_IBEO_H_
