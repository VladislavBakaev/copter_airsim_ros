#ifndef COPTERCONTROL_H_
#define COPTERCONTROL_H_

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>


class CopterControl {

public:
	// public elements
	CopterControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

private:
	double publish_odom_every_n_sec = 0.1;

	void initialize_ros();
	void gps_cb(const sensor_msgs::NavSatFix& gps_msg);
	void imu_cb(const sensor_msgs::Imu& imu_msg);
	void publish_odom_cb(const ros::TimerEvent& event);

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	ros::Subscriber gps_sub_;
	ros::Subscriber imu_sub_;

	nav_msgs::Odometry curr_odom_;

	ros::Timer publish_odom_timer_;
};

#endif