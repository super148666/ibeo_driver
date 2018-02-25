#include "lidar_manager.h"
#include <thread>



int main(int argc, char** argv)
{
	ros::init(argc, argv, "ibeo_driver");
	ros::NodeHandle nh;
	lidar_manager* lm = new lidar_manager(nh);
	lm->setup();
	for(int i=0;i<10;i++) {
		lm->update();
		if(lm->has_scan_data()) {
			ROS_INFO_STREAM("new scan data available");
			ScanData sd = lm->get_scan_data();
			ROS_INFO_STREAM(" " << sd.num_scan_points << " scan points available");
		}
		sleep(1);
	}
	lm->exit();
	return 0;
}
