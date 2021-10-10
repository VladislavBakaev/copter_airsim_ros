#!/usr/bin/python3

import rospy
from geodetic_conv import GeodeticConvert
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class DataComplexator():
    def __init__(self, vehicle_name) -> None:
        rospy.Subscriber('/'+vehicle_name+'/ins_nav_topic', NavSatFix, self.gps_data_cb)
        self.odom_publisher = rospy.Publisher('/airsim_node/'+vehicle_name+'/odom_local_ned', Odometry, queue_size=10)
        self.geodetic_convertor = GeodeticConvert()
        self.gps_msg = NavSatFix()
        self.odom_msg = Odometry()
        rospy.Timer(rospy.Duration(0.1), self.publish_odometry)
        rospy.Timer(rospy.Duration(0.1), self.complexation_data)

    def complexation_data(self, event):
        
        n, e, d = self.geodetic_convertor.geodetic2Ned(self.gps_msg.latitude,\
                                                       self.gps_msg.longitude,\
                                                       self.gps_msg.altitude)

        # complexation

        x = n
        y = e
        z = d

        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.pose.pose.position.x = x
        self.odom_msg.pose.pose.position.y = y
        self.odom_msg.pose.pose.position.z = z

    def publish_odometry(self, event):
        self.odom_publisher.publish(self.odom_msg)

    def gps_data_cb(self, msg):
        if not self.geodetic_convertor.isInitialised():
            self.geodetic_convertor.initialiseReference(msg.latitude,\
                                                        msg.longitude,\
                                                        msg.altitude)
        self.gps_msg = msg

if __name__=="__main__":
    rospy.init_node('complexation_node')
    vehicle_name = ""

    while(vehicle_name == ""):
        vehicle_name = rospy.get_param('/vehicle_name', vehicle_name)
        rospy.loginfo("Wait vechical name")

    data_complexator = DataComplexator(vehicle_name)

    rospy.spin()