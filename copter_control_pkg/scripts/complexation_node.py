#!/usr/bin/python3

import rospy
from geodetic_conv import GeodeticConvert
from sensor_msgs.msg import NavSatFix, Imu, Range
from nav_msgs.msg import Odometry
from kalmanFilter import KalmanFilterComplex
from airsim_ros_pkgs.msg import GPSYaw
from tf.transformations import euler_from_quaternion
from imu_integrate import ImuData
from std_msgs.msg import Bool

class DataComplexator():
    def __init__(self, vehicle_name) -> None:
        rospy.Subscriber('/'+vehicle_name+'/ins_nav_topic', NavSatFix, self.gps_data_cb)
        rospy.Subscriber('/'+vehicle_name+'/ins_imu_topic', Imu, self.imu_data_cb)
        rospy.Subscriber('/'+vehicle_name+'/laser_dist_topic', Range, self.laser_data_cb)
        rospy.Subscriber('/'+vehicle_name+'/aion_nav_topic', NavSatFix, self.aion_data_cb)
        rospy.Subscriber('/'+vehicle_name+'/antispoof_topic', Bool, self.antispoof_cb)

        self.antispoof = False

        self.odom_publisher = rospy.Publisher('/airsim_node/'+vehicle_name+'/odom_local_ned', Odometry, queue_size=10)
        self.set_home = rospy.Publisher('/airsim_node/home_geo_point', GPSYaw, queue_size=10)
        self.geodetic_convertor = GeodeticConvert()
        self.gps_msg = NavSatFix()
        self.odom_msg = Odometry()
        self.range_msg = Range()
        self.aion_msg = NavSatFix()
        self.imu_msg = Imu()
        self.imu_moving = ImuData()

        self.filter_gps = KalmanFilterComplex()
        self.filter_aion = KalmanFilterComplex()
        for i in range(3):
            self.filter_aion.R[i,i] = 2

        rospy.Timer(rospy.Duration(0.1), self.publish_odometry)
        rospy.Timer(rospy.Duration(0.05), self.complexation_data)

    def complexation_data(self, event):
        
        if self.geodetic_convertor.isInitialised():
            n, e, d = self.geodetic_convertor.geodetic2Ned(self.gps_msg.latitude,\
                                                        self.gps_msg.longitude,\
                                                        self.gps_msg.altitude)

            n_aion, e_aion, d_aion = self.geodetic_convertor.geodetic2Ned(self.aion_msg.latitude,\
                                            self.aion_msg.longitude,\
                                            self.gps_msg.altitude)

            # complexation

            rpy = euler_from_quaternion([self.imu_msg.orientation.x,
                                        self.imu_msg.orientation.y,\
                                        self.imu_msg.orientation.z,\
                                        self.imu_msg.orientation.w])
            
            self.filter_gps.kalmanUpdate([n-float(self.imu_moving.northIns),\
                                      0,e-float(self.imu_moving.eastIns)], rpy)

            self.filter_aion.kalmanUpdate([n_aion-float(self.imu_moving.northIns),\
                            0,e_aion-float(self.imu_moving.eastIns)], rpy)

            x = n
            y = e
            # z = d

            self.odom_msg.header.stamp = rospy.Time.now()
            if not self.antispoof:
                self.odom_msg.pose.pose.position.x = self.filter_gps.xErr[0]*0.5 + x
                self.odom_msg.pose.pose.position.y = self.filter_gps.xErr[2]*0.5 + y
            else:
                self.odom_msg.pose.pose.position.x = self.filter_aion.xErr[0]*0.5 + x
                self.odom_msg.pose.pose.position.y = self.filter_aion.xErr[2]*0.5 + y 
            self.odom_msg.pose.pose.position.z = -self.range_msg.range
            self.odom_msg.pose.pose.orientation = self.imu_msg.orientation

    def publish_odometry(self, event):
        self.odom_publisher.publish(self.odom_msg)

    def gps_data_cb(self, msg):
        if not self.geodetic_convertor.isInitialised():
            self.geodetic_convertor.initialiseReference(msg.latitude,\
                                                        msg.longitude,\
                                                        msg.altitude)
            home = GPSYaw()
            for i in range(30):
                home.latitude = msg.latitude
                home.longitude = msg.longitude
                home.altitude = msg.altitude
                home.yaw = 0.0
                self.set_home.publish(home)
                rospy.sleep(0.01)

        self.gps_msg = msg

    def imu_data_cb(self, msg):
        self.imu_msg = msg
        self.imu_moving.update_date([msg.linear_acceleration.x,\
                                    msg.linear_acceleration.y,\
                                    msg.linear_acceleration.z])

    def aion_data_cb(self, msg):
        self.aion_msg = msg

    def laser_data_cb(self, msg):
        self.range_msg = msg

    def antispoof_cb(self, msg):
        self.antispoof = msg.data

if __name__=="__main__":
    rospy.init_node('complexation_node')
    vehicle_name = ""

    while(vehicle_name == ""):
        vehicle_name = rospy.get_param('/vehicle_name', vehicle_name)
        rospy.loginfo("Wait vechical name")

    data_complexator = DataComplexator(vehicle_name)

    rospy.spin()