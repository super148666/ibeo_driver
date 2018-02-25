#include "ibeo_driver/ibeo.h"
#include "ibeo_driver/ibeo_network.h"
#include "reader.h"
#include <thread>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ibeo_driver");
	ros::NodeHandle nh;
	IBEO* ibeo = new IBEO(nh);
	ibeo->Open();
	ibeo->Start();
	sleep(10);
	delete ibeo;
	
	return 0;
}
