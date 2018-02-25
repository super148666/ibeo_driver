#include "lidar_manager.h"
#include <glog/logging.h>
#include <math.h>

lidar_manager::lidar_manager(ros::NodeHandle nh)  : _io_service(), lidar_reader(this->_io_service,nh) {
}


lidar_manager::~lidar_manager() {
  exit();
}

// PUBLIC /////////////////////////////////////////////////////////////////////
void lidar_manager::setup() {
  initialise_data_stream();
}

void lidar_manager::exit() {
  lidar_reader.close();
  if (io_thread.joinable()) io_thread.join();
}

void lidar_manager::update() {
  read_data_from_lidar();
}


ScanData lidar_manager::get_scan_data(){
	has_new_scan_data = false;
	return current_scan_data;
}

bool lidar_manager::has_scan_data(){
	return has_new_scan_data;
}


// READING DATA ////////////////

void lidar_manager::initialise_data_stream() {
    // Start io service thread to continuously update world_objects.
    io_thread = std::thread(&lidar_manager::io_service_thread, this);
}

void lidar_manager::io_service_thread() { _io_service.run(); }


// Reads in data from the LiDAR.
// Saves ALL LiDAR data to a binary protobuf file
// Stores the most recent object data and scan data.
// 
// record_all_data toggles between storing everything the LiDAR captures 
// or storing only the object/scan data that the program reads during
// runtime.
//  This is to avoid the problem of the LiDAR potential suplying more
//  data then the program can keep up with.

void lidar_manager::read_data_from_lidar() {

    bool record_all_data =true;

    std::vector<LidarMessage> messages;

    // Store the msot recent object data and scan data
    while (!lidar_reader._messages.empty()) {

      auto msg = lidar_reader._messages.back();
      messages.push_back(msg);
      lidar_reader._messages.pop_back();

      bool found_scan_data = false;
      bool found_object_data = false;

      if (msg.scan_data.num_scan_points!=0 && !found_scan_data) {
        current_scan_data = msg.scan_data;
        /*
        if (!record_all_data) {
			write_message_to_file(msg);
        }
		*/
		ROS_INFO_STREAM("found scan data");
        found_scan_data = true;
        has_new_scan_data = true;
      }
    }

    while (record_all_data && !messages.empty()) {
      auto msg = messages.front();
      /*
      write_message_to_file(msg);
      */
      ROS_INFO_STREAM("found other data");
      messages.erase(messages.begin());
    }
    ROS_INFO_STREAM("no data recieved");
}

