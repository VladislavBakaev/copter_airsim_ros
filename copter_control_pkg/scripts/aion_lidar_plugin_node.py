#!/usr/bin/python3

import rospy
from sensor_msgs.msg import NavSatFix, Range

gps_data = NavSatFix()

def gps_data_cb(msg) -> None:
    global gps_data
    gps_data = msg

class AionPlugin():
    def __init__(self, vehicle_name, rate) -> None:

        self.aion_publisher = rospy.Publisher('/airsim_node/'+vehicle_name+'/aion/Aion', NavSatFix, queue_size=10)
        rospy.Timer(rospy.Duration(rate), self.aion_data_publisher_timer)

    def aion_data_publisher_timer(self, event) -> None:
        new_msg = NavSatFix()
        new_msg.longitude = gps_data.longitude
        new_msg.latitude = gps_data.latitude
        new_msg.header.stamp = rospy.Time.now()
        new_msg.altitude = 0.0
        self.aion_publisher.publish(new_msg)

class LidarDistancePlugin():
    def __init__(self, vehicle_name, rate) -> None:
        
        self.lidar_publisher = rospy.Publisher('/airsim_node/'+vehicle_name+'/distance/Distance', Range, queue_size=10)
        rospy.Timer(rospy.Duration(rate), self.lidar_data_publisher_timer)
        self.range_msg = Range()
        self.range_msg.min_range = 0.0
        self.range_msg.max_range = 40.0
        self.altitude_offset = 119.92363739013672

    def lidar_data_publisher_timer(self, event) ->None:
        self.range_msg.header.stamp = rospy.Time.now()
        self.range_msg.range = gps_data.altitude - self.altitude_offset
        self.lidar_publisher.publish(self.range_msg)

if __name__=='__main__':    
    rospy.init_node('aion_lidar_plugin_node')
    aion_rate = 1
    lidar_rate = 10
    vehicle_name = ""

    while(vehicle_name == ""):
        vehicle_name = rospy.get_param('/vehicle_name', vehicle_name)
        rospy.loginfo("Wait vechical name")
    
    aionPlugin = AionPlugin(vehicle_name, 1/aion_rate)
    # lidarPlugin = LidarDistancePlugin(vehicle_name, 1/lidar_rate)
    
    rospy.Subscriber('/airsim_node/'+vehicle_name+"/gps/Gps", NavSatFix, gps_data_cb)
    rospy.spin()
