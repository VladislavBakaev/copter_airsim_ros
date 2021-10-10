#!/usr/bin/python3

import rospy
import rospkg
import json

if __name__=="__main__":
    rospack = rospkg.RosPack()

    pkg_path = rospack.get_path('copter_control_pkg')

    with open(pkg_path+'/config/config.json', 'r') as json_file:
        data = json.load(json_file)
        rospy.set_param('lidar_rate', data['lidar_rate'])
        rospy.set_param('lidar_accuracy', data['lidar_accuracy'])
        rospy.set_param('aion_rate', data['aion_rate'])
        rospy.set_param('aion_accuracy', data['aion_accuracy'])
        rospy.set_param('gps_rate', data['gps_rate'])
        rospy.set_param('gps_accuracy', data['gps_accuracy'])
        rospy.set_param('imu_rate', data['imu_rate'])
        rospy.set_param('imu_orientation_accuracy', data['imu_orientation_accuracy'])
        rospy.set_param('imu_angular_velocity_accuracy', data['imu_angular_velocity_accuracy'])
        rospy.set_param('imu_linear_acceleration_accuracy', data['imu_linear_acceleration_accuracy'])

        