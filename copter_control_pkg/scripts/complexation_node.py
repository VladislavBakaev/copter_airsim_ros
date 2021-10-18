#!/usr/bin/python3

from decimal import Decimal
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
        self.antispoof = False
        rospy.loginfo('Wait controller node ...')
        rospy.wait_for_service('/airsim_node/gps_goal')
        rospy.loginfo('Controller node is running')

        self.odom_publisher = rospy.Publisher('/airsim_node/'+vehicle_name+'/odom_local_ned', Odometry, queue_size=10)
        self.odom_not_filter_publisher = rospy.Publisher('/airsim_node/'+vehicle_name+'/odom_local_ned_not_filter', Odometry, queue_size=10)
        self.set_home = rospy.Publisher('/airsim_node/home_geo_point', GPSYaw, queue_size=10)
        self.geodetic_convertor = GeodeticConvert()
        self.gps_msg = NavSatFix()
        self.odom_msg = Odometry()
        self.odom_not_filter_msg = Odometry()
        self.range_msg = Range()
        self.aion_msg = NavSatFix()
        self.imu_msg = Imu()
        self.imu_moving = ImuData()
        self.ned = [0,0,0]
        self.filter_rate = 0.01

        self.filter_gps = KalmanFilterComplex()
        self.filter_aion = KalmanFilterComplex()
        self.filter_gps.dt = self.filter_rate
        self.filter_aion.dt = self.filter_rate
        for i in range(3):
            self.filter_aion.R[i,i] = 10

        rospy.Subscriber('/'+vehicle_name+'/ins_nav_topic', NavSatFix, self.gps_data_cb)
        rospy.Subscriber('/'+vehicle_name+'/ins_imu_topic', Imu, self.imu_data_cb)
        rospy.Subscriber('/'+vehicle_name+'/laser_dist_topic', Range, self.laser_data_cb)
        rospy.Subscriber('/'+vehicle_name+'/aion_nav_topic', NavSatFix, self.aion_data_cb)
        rospy.Subscriber('/'+vehicle_name+'/antispoof_topic', Bool, self.antispoof_cb)
        rospy.Subscriber('/airsim_node/'+vehicle_name+'/gps/Gps', NavSatFix, self.ideal_gps_cb)

        rospy.Timer(rospy.Duration(0.1), self.publish_odometry)
        rospy.Timer(rospy.Duration(self.filter_rate), self.complexation_data)

    def complexation_data(self, event):
        
        if self.geodetic_convertor.isInitialised():
            n, e, d = self.geodetic_convertor.geodetic2Ned(self.gps_msg.latitude,\
                                                        self.gps_msg.longitude,\
                                                        self.gps_msg.altitude)

            self.ned = [n,e,d]

            n_aion, e_aion, d_aion = self.geodetic_convertor.geodetic2Ned(self.aion_msg.latitude,\
                                            self.aion_msg.longitude,\
                                            self.gps_msg.altitude)

            # complexation

            rpy = euler_from_quaternion([self.imu_msg.orientation.x,
                                        self.imu_msg.orientation.y,\
                                        self.imu_msg.orientation.z,\
                                        self.imu_msg.orientation.w])
            
            self.filter_gps.kalmanUpdate([n,\
                                          self.range_msg.range,\
                                          e], rpy)

            self.filter_aion.kalmanUpdate([n_aion,\
                                           self.range_msg.range,\
                                           e_aion], rpy)

            # x = n
            # y = e
            # z = d

            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_not_filter_msg.header.stamp = rospy.Time.now()
            if not self.antispoof:
                self.odom_msg.pose.pose.position.x = self.filter_gps.xErr[0]
                self.odom_msg.pose.pose.position.y = self.filter_gps.xErr[2]
                self.odom_not_filter_msg.pose.pose.position.x = n
                self.odom_not_filter_msg.pose.pose.position.y = e
            else:
                self.odom_msg.pose.pose.position.x = self.filter_aion.xErr[0]
                self.odom_msg.pose.pose.position.y = self.filter_aion.xErr[2]
                self.odom_not_filter_msg.pose.pose.position.x = n_aion
                self.odom_not_filter_msg.pose.pose.position.y = e_aion
            self.odom_msg.pose.pose.position.z = -self.filter_aion.xErr[1]
            self.odom_msg.pose.pose.orientation = self.imu_msg.orientation
            self.odom_not_filter_msg.pose.pose.position.z = -self.range_msg.range
            self.odom_not_filter_msg.pose.pose.orientation = self.imu_msg.orientation

    def publish_odometry(self, event):
        self.odom_publisher.publish(self.odom_msg)
        self.odom_not_filter_publisher.publish(self.odom_not_filter_msg)

    def ideal_gps_cb(self, msg):
        if not self.geodetic_convertor.isInitialised():
            self.geodetic_convertor.initialiseReference(msg.latitude,\
                                                        msg.longitude,\
                                                        msg.altitude)
            self.filter_gps.f = msg.latitude
            self.filter_gps.f = msg.latitude
            home = GPSYaw()
            for i in range(50):
                home.latitude = msg.latitude
                home.longitude = msg.longitude
                home.altitude = msg.altitude
                home.yaw = 0.0
                self.set_home.publish(home)
                rospy.sleep(0.01)

    def gps_data_cb(self, msg):
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
    rospy.init_node('complexation_node', disable_signals=True)
    rospy.loginfo('Start complexation node')
    vehicle_name = ""

    while(vehicle_name == ""):
        vehicle_name = rospy.get_param('/vehicle_name', vehicle_name)
        rospy.loginfo("Wait vechical name (complexation node)")

    data_complexator = DataComplexator(vehicle_name)

    rospy.spin()