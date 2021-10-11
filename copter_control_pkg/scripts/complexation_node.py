#!/usr/bin/python3

import rospy
from geodetic_conv import GeodeticConvert
from sensor_msgs.msg import NavSatFix, Imu, Range
from nav_msgs.msg import Odometry
from kalmanFilter import KalmanFilterComplex
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class DataComplexator():
    def __init__(self, vehicle_name) -> None:
        rospy.Subscriber('/'+vehicle_name+'/ins_nav_topic', NavSatFix, self.gps_data_cb)
        rospy.Subscriber('/'+vehicle_name+'/ins_imu_topic', Imu, self.imu_data_cb)
        rospy.Subscriber('/'+vehicle_name+'/laser_dist_topic', Range, self.laser_data_cb)
        rospy.Subscriber('/'+vehicle_name+'/aion_nav_topic', NavSatFix, self.aion_data_cb)

        self.odom_publisher = rospy.Publisher('/airsim_node/'+vehicle_name+'/odom_local_ned', Odometry, queue_size=10)
        self.geodetic_convertor = GeodeticConvert()
        self.gps_msg = NavSatFix()
        self.odom_msg = Odometry()
        self.range_msg = Range()
        self.aion_msg = NavSatFix()
        self.imu_msg = Imu()

        self.filter = KalmanFilterComplex()

        rospy.Timer(rospy.Duration(0.1), self.publish_odometry)
        rospy.Timer(rospy.Duration(0.05), self.complexation_data)

    def complexation_data(self, event):
        
        if self.geodetic_convertor.isInitialised():
            n, e, d = self.geodetic_convertor.geodetic2Ned(self.gps_msg.latitude,\
                                                        self.gps_msg.longitude,\
                                                        self.gps_msg.altitude)

            # complexation

            rpy = euler_from_quaternion([self.imu_msg.orientation.x,
                                        self.imu_msg.orientation.y,\
                                        self.imu_msg.orientation.z,\
                                        self.imu_msg.orientation.w])
            
            self.filter.kalmanUpdate([n,self.range_msg.range,e], rpy)

            x = n
            y = e
            # z = d

            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.pose.pose.position.x = self.filter.xErr[0]
            self.odom_msg.pose.pose.position.y = self.filter.xErr[2]
            self.odom_msg.pose.pose.position.z = -self.filter.xErr[1]
            self.odom_msg.pose.pose.orientation = self.imu_msg.orientation

    def publish_odometry(self, event):
        self.odom_publisher.publish(self.odom_msg)

    def gps_data_cb(self, msg):
        if not self.geodetic_convertor.isInitialised():
            self.geodetic_convertor.initialiseReference(msg.latitude,\
                                                        msg.longitude,\
                                                        msg.altitude)
        self.gps_msg = msg

    def imu_data_cb(self, msg):
        self.imu_msg = msg

    def aion_data_cb(self, msg):
        self.aion_msg = msg

    def laser_data_cb(self, msg):
        self.range_msg = msg


if __name__=="__main__":
    rospy.init_node('complexation_node')
    vehicle_name = ""

    while(vehicle_name == ""):
        vehicle_name = rospy.get_param('/vehicle_name', vehicle_name)
        rospy.loginfo("Wait vechical name")

    data_complexator = DataComplexator(vehicle_name)

    rospy.spin()