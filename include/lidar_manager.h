#ifndef SENSOR_MANAGER_LIDAR_MANAGER_LIDAR_MANAGER_H_
#define SENSOR_MANAGER_LIDAR_MANAGER_LIDAR_MANAGER_H_

#include "reader.h"
#include "boost/math/constants/constants.hpp"

#include <vector>
#include <thread>
#include <iostream>
#include <string>

#define boostPi boost::math::constants::pi<double>()

class lidar_manager {
public:

lidar_manager(ros::NodeHandle nh);

void setup();
void exit();

void update();

ScanData get_scan_data();
bool has_scan_data();

virtual ~lidar_manager();

private:
	LidarMessage lidar_message;
	std::vector<LidarMessage> lidar_message_list;
	LidarMessage current_lidar_message;
	ObjectData current_object_data;
	ScanData current_scan_data;

	boost::asio::io_service _io_service;
	Reader lidar_reader;
	std::thread io_thread;

	bool has_new_object_data = true;
	bool has_new_scan_data = true;
	
	int freq_check = 0;

	void initialise_data_stream();
	void io_service_thread();
	
	int message_count;
	int num_scan_data;
  	int scan_counter;

	void read_data_from_lidar();
};

#endif // SENSOR_MANAGER_LIDAR_MANAGER_LIDAR_MANAGER_H_

