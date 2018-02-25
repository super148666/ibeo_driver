/* Copyright 2013 Thomas Drage
 * File:   IBEO.cpp
 * Author: Timothy Black (20373081), modifications Thomas Drage (20510505).
 *
 * Created on 18 April 2012, 7:34 PM
 * Modified 3 March 2013, 23 July 2013 (T.Drage).
 *
 * Road finding and object projection T. Drage Sep 2013.
 */

#include "ibeo_driver/ibeo.h"

#define SOCKET_FD connection->socketFD

    static char ibeo_magic_word[] = {
            static_cast<uint8_t>(0xAF), static_cast<uint8_t>(0xFE), static_cast<uint8_t>(0xC0),
            static_cast<uint8_t>(
                    0xC2)}; // The magic word to look for for beginning of message.

    int curScanDataSource = 0;   // Current scan data source/buffer.
    int curObjectDataSource = 0; // Current object data source/buffer.
    int curErrorDataSource = 0;  // Current error data source/buffer.

    bool gotObject = false;
    bool gotScan = false;

    bool verbose = true;

    char byteIn = 0;

/**
 * Purpose: Creates a new instance of the IBEO object.
 * Inputs : None.
 * Outputs: None.
 */
    IBEO::IBEO(ros::NodeHandle nh):
    nh(nh)
    {
        gettimeofday(&lastwrite, NULL);
        layersToScan = LAYERS;

        connection = new IBEONetwork();

        for (int i = 0; i < MSG_BUFFERS; i++) {
            scan_data_header[i].scan_points = 0;
            object_data_header[i].number_of_objects = 0;
        }
        inUse = false;
        laserscan.header.frame_id = "laser_frame";
        laserscan.range_min = 0.3;
        laserscan.range_max = 200.0;

        scan_pub = nh.advertise<sensor_msgs::LaserScan>("laserscan",50);
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
        ROS_INFO_STREAM("IBEO scanner - attempting to connect to %s : %d ." +
                        boost::lexical_cast<std::string>(ip_addr) + ":" +
                        boost::lexical_cast<std::string>(port) + ".");
        if (!connection->Connect(ip_addr, port)) {
            ROS_INFO_STREAM("IBEO scanner - Connecting failed...");
            return false;
        }
        inUse = true;
        ROS_INFO_STREAM("IBEO scanner - connected to scanner successfully.");

        return true;
    }

/**
 * Purpose: Connects to the ibeo scanner using the default IP Address and port.
 * Inputs : None.
 * Outputs: true if successful, false otherwise.
 */
    bool IBEO::Open() { return Open((char *) IBEO_IP_ADDRESS, IBEO_PORT); }

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
            ROS_INFO_STREAM("ibeo scanner: disconnected from scanner.");
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
                    ROS_INFO_STREAM("Caught 1");
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
            ROS_INFO_STREAM("ibeo scanner: found header for device id "
                                    << htons(headerIn.dev_id) << ".");
            ROS_INFO_STREAM("              data type for next message is "
                                    << htons(headerIn.data_type) << ".");
            ROS_INFO_STREAM("              size of next message is "
                                    << htons(headerIn.size_cur_msg) << ".");
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
                 gotScan == true)) { // Scan until we have one of each set of data
            header = this->FindHeader();
            try {
				switch (htons(header.data_type)) {
					case 0x2202:
						if (verbose)
							ROS_INFO_STREAM("ibeo scanner: found scan data.");
						if (!Read_Scan_Data())
							return;
						break;
					case 0x2221:
						if (verbose)
							ROS_INFO_STREAM("ibeo scanner: found object data.");
						if (!Read_Object_Data())
							return;
						break;
					case 0x2030:
						if (verbose)
							ROS_INFO_STREAM("ibeo scanner: found error data.");
						if (!Read_Errors())
							return;
						break;
					default:
						if (verbose)
							ROS_INFO_STREAM(
									"ibeo scanner: received unknown message with id ");
						break;
				}
			}
			catch (int e) {
				ROS_INFO_STREAM("IBEO - caught error!");
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

        next_scan_source = (curScanDataSource + 1) % 2;
        if (Recv(SCAN_HEADER.scan_number))
            return false;
        if (Recv(SCAN_HEADER.scanner_status))
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
//		ROS_INFO_STREAM("angle_min_raw:"<<SCAN_HEADER.start_angle);
//		ROS_INFO_STREAM("angle_max_raw:"<<SCAN_HEADER.stop_angle);
//        laserscan.angle_min = ConvertTicktsToAngle(SCAN_HEADER.start_angle);
//        laserscan.angle_max = ConvertTicktsToAngle(SCAN_HEADER.stop_angle);
        laserscan.angle_min = -50/180*M_PI;
        laserscan.angle_max = 50/180*M_PI;
        laserscan.angle_increment = ConvertTicktsToAngle(1);
        if (verbose)
        {
			ROS_INFO_STREAM("ang tick per rev " << SCAN_HEADER.angle_ticks_per_rotation);
			ROS_INFO_STREAM("scan status " << SCAN_HEADER.scanner_status);
			ROS_INFO_STREAM("scan num " << SCAN_HEADER.scan_number);
            ROS_INFO_STREAM("scan points " << SCAN_HEADER.scan_points);
            ROS_INFO_STREAM("start angle " << SCAN_HEADER.start_angle);
            ROS_INFO_STREAM("stop angle " << SCAN_HEADER.stop_angle);
                                                     
        }

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
            laserscan.ranges[SCAN_POINTS.horiz_angle] = SCAN_POINTS.radial_dist/100;
            ROS_INFO_STREAM("ticks:"<<SCAN_POINTS.horiz_angle<<" dist:"<<SCAN_POINTS.radial_dist);
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
            ROS_INFO_STREAM("ibeo scanner: reading " << OBJ_HEADER.number_of_objects
                                                     << " objects from ibeo.");

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
            ROS_INFO_STREAM("ibeo scanner: receiving errors from the ibeo.");

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
    template<class T>
    inline bool IBEO::Recv(T msg) {
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
            if ((current.tv_sec + ((double) current.tv_usec) / 1000000) >
                (lastwrite.tv_sec + ((double) lastwrite.tv_usec) / 1000000) +
                IBEO_PERIOD) {
                if(gotScan) {
                    scan_pub.publish(laserscan);
                    ROS_INFO_STREAM("pub success!");
                }
                gettimeofday(&lastwrite, NULL);
                //      ROS_INFO_STREAM("After Get Time of Day...");
            }
        }
    }

    float IBEO::ConvertTicktsToAngle(int16_t angleTicks)
    {
        float angle = static_cast<float>(((float)angleTicks / 32.0) * (M_PI / 180));

        if ((angle > -1000.0) && (angle < 1000.0))
        {
            // Angle ist in "sinnvollem" Bereich
            while (angle > M_PI)
            {
                angle -= 2.0 * M_PI;
            }
            while (angle <= -M_PI)
            {
                angle += 2.0 * M_PI;
            }
        }
        else
        {
            // Winkel ist unsinning
            ROS_ERROR_STREAM("convertTicktsToAngle(): The angle value " << angle << " is widely out of the reasonable range, setting to 0.0.");
            angle = 0.0;
        }

        return angle;
    }



/*
int main(int argc, char **argv) {
    ros::init(argc, argv, "ibeo_driver_node");
    ros::NodeHandle nh;
    lidar::IBEO ibeo = lidar::IBEO(nh);
    ibeo.Open();
    ibeo.Start();
    sleep(30);
    ibeo.StopScanner();
    return 0;
}
*/
