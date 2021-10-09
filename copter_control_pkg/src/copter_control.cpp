#include "copter_control.h"

CopterControl::CopterControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	:nh_(nh), nh_private_(nh_private)
{
    initialize_ros();
}

void CopterControl::initialize_ros() {
    std::string vehicle_name;

    while (vehicle_name == "") {
        nh_private_.getParam("/vehicle_name", vehicle_name);
        ROS_INFO_STREAM("Waiting vehicle name");
    }

    gps_sub_ = nh_.subscribe("/airsim_node/" + vehicle_name + "/gps/Gps", 50, &CopterControl::gps_cb, this);
    imu_sub_ = nh_.subscribe("/airsim_node/" + vehicle_name + "/imu/Imu", 50, &CopterControl::imu_cb, this);

    publish_odom_timer_ = nh_private_.createTimer(ros::Duration(publish_odom_every_n_sec), &CopterControl::publish_odom_cb, this);
}

void CopterControl::gps_cb(const sensor_msgs::NavSatFix& gps_msg) {
    
}

void CopterControl::imu_cb(const sensor_msgs::Imu& imu_msg) {

}

void CopterControl::publish_odom_cb(const ros::TimerEvent& event) {
    ROS_INFO_STREAM("timer");
}