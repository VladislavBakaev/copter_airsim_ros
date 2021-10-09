#include "ros/ros.h"
#include "copter_control.h"
  

int main(int argc, char** argv)
{
	ros::init(argc, argv, "copter_control_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	CopterControl controller(nh, nh_private);

	ros::spin();

	return 0;
}