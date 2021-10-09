#!/usr/bin/python3

from enum import Flag
from sys import flags
import rospy
from sensor_msgs.msg import NavSatFix, Range, Imu
import numpy as np
from math import pi, sqrt, sin, cos

class GeodeticParam():
    def __init__(self) -> None:
        self.a = 6378206.4
        self.b = 6356583.8
        self.e_2 = 1 - self.b**2/self.a**2

    def getCurrentRadius(self, lat, lon, alt) -> float:
        N = self.a/(sqrt(1-self.e_2*sin(lat)**2))

        x = (N + alt)*cos(lat)*cos(lon)
        y = (N + alt)*cos(lat)*sin(lon)

        z = (self.b**2/self.a**2*N + alt)*sin(lat)

        return sqrt(x**2 + y**2 + z**2)


class ParametersLoader():
    def __init__(self) -> None:
        flag = True
        rospy.loginfo('Loading parameters')
        while flag:
            self.lidar_rate = rospy.get_param('lidar_rate', False)
            self.lidar_accuracy = rospy.get_param('lidar_accuracy', False)
            self.aion_rate = rospy.get_param('aion_rate', False)
            self.aion_accuracy = rospy.get_param('aion_accuracy', False)
            self.gps_rate = rospy.get_param('gps_rate', False)
            self.gps_accuracy = rospy.get_param('gps_accuracy', False)
            self.imu_rate = rospy.get_param('imu_rate', False)
            self.imu_orientation_accuracy = rospy.get_param('imu_orientation_accuracy', False)
            self.imu_angular_velocity_accuracy = rospy.get_param('imu_angular_velocity_accuracy', False)
            self.imu_linear_acceleration_accuracy = rospy.get_param('imu_linear_acceleration_accuracy', False)
            if(self.lidar_rate and self.lidar_accuracy and self.aion_rate and\
                self.aion_accuracy and self.gps_rate and self.gps_accuracy and\
                self.imu_rate and self.imu_angular_velocity_accuracy and\
                self.imu_linear_acceleration_accuracy and self.imu_orientation_accuracy):
                flag = False
        rospy.loginfo('Param loaded')


class LidarNoizer():
    def __init__(self, vehicle_name, params) -> None:
        rospy.Subscriber('/airsim_node/'+vehicle_name+'/distance/Distance', Range, self.laser_data_cb)
        self.publisher = rospy.Publisher('/'+vehicle_name+'/laser_dist_topic', Range, queue_size=10)

        self.rate = 1/params.lidar_rate

        rospy.Timer(rospy.Duration(self.rate), self.publish_laser_data)

        self.range_ = 0.0
        self.accuracy = params.lidar_accuracy
        self.msg = Range()

        #probabilistic param
        self.w1 = 0.85
        self.w2 = 0.97
        self.w3 = 1

    def laser_data_cb(self, msg) ->None:
        self.msg = msg
        # self.publisher.publish(msg)
    
    def publish_laser_data(self, event) -> None:
        range = self.probabilisticModel(self.msg.range, 500, 0) # error with distance data from airsim. Now i am using a kostyl
        new_msg = Range()
        new_msg.header = self.msg.header
        new_msg.max_range = self.msg.max_range
        new_msg.min_range = self.msg.min_range
        new_msg.range = range
        self.publisher.publish(new_msg)

    def probabilisticModel(self, range_, max_distance, min_distance) -> Range:
        range_new = 0.0
        w = np.random.rand()
        if w < self.w1:
            range_new = np.random.normal(range_, self.accuracy/3)
        elif w < self.w2:
            range_new = (range_-min_distance)*np.random.rand() + min_distance
        else:
            range_new = 5*np.random.rand() + max_distance-5

        return range_new


class AionNoizer():
    def __init__(self, vehicle_name, geo_param, params) -> None:
        rospy.Subscriber('/airsim_node/'+vehicle_name+'/aion/Aion', NavSatFix, self.aion_data_cb)
        self.publisher = rospy.Publisher('/'+vehicle_name+'/aion_nav_topic', NavSatFix, queue_size=10)

        self.rate = 1/params.aion_rate

        rospy.Timer(rospy.Duration(self.rate), self.publish_aion_data)
        self.msg = NavSatFix()

        self.geo_param = geo_param
        self.accuracy = params.aion_accuracy
        self.accuracy_blowout_k = 2
        self.acuuracy_max_blowout_k = 3
        self.w1 = 0.5
        self.w2 = 0.9
        self.w3 = 1

    def aion_data_cb(self, msg) ->None:
        self.msg = msg

    def publish_aion_data(self, event) -> None:
        lat, lon = self.probabilisticModel(self.msg)
        new_msg = NavSatFix()
        new_msg.header = self.msg.header
        new_msg.latitude = lat
        new_msg.longitude = lon
        new_msg.altitude = 0.0
        self.publisher.publish(new_msg)

    def probabilisticModel(self, msg) -> list:

        err = [0, 0]
        accuracy_grad = np.array(self.accuracy)/self.geo_param.getCurrentRadius(msg.latitude, msg.longitude, msg.altitude)*180/pi

        for i, _ in enumerate(err):
            w = np.random.rand()
            if w < self.w1:
                err[i] = np.random.normal(0, accuracy_grad[i]/3)
            elif w < self.w2:
                err[i] = 2*self.accuracy_blowout_k*accuracy_grad[i]*np.random.rand()-self.accuracy_blowout_k*accuracy_grad[i]
            else:
                err[i] = 2*self.acuuracy_max_blowout_k*accuracy_grad[i]*np.random.rand()-self.acuuracy_max_blowout_k*accuracy_grad[i]

        return [msg.latitude + err[0], msg.longitude + err[1]]


class GpsNoizer():
    def __init__(self, vehicle_name, geo_param, params) -> None:
        rospy.Subscriber('/airsim_node/'+vehicle_name+'/gps/Gps', NavSatFix, self.gps_data_cb)
        self.publisher = rospy.Publisher('/'+vehicle_name+'/ins_nav_topic', NavSatFix, queue_size=10)
        self.geo_param = geo_param

        self.rate = 1/params.gps_rate

        rospy.Timer(rospy.Duration(self.rate), self.publish_gps_data)
        self.msg = NavSatFix()

        self.accuracy = params.gps_accuracy

        self.w1 = 0.5
        self.w2 = 0.9
        self.w3 = 1
        self.accuracy_blowout_k = [2, 2, 0]
        self.acuuracy_max_blowout_k = [3, 3, 0]

    def gps_data_cb(self, msg) ->None:
        self.msg = msg

    def publish_gps_data(self, event) -> None:
        lat, lon, alt = self.probabilisticModel(self.msg)
        new_msg = NavSatFix()
        new_msg.header = self.msg.header
        new_msg.latitude = lat
        new_msg.longitude = lon
        new_msg.altitude = alt
        self.publisher.publish(new_msg)

    def probabilisticModel(self, msg) -> list:

        err = [0, 0, 0]

        accuracy_grad = np.array(self.accuracy[:2])/self.geo_param.getCurrentRadius(msg.latitude, msg.longitude, msg.altitude)*180/pi
        accuracy_grad = accuracy_grad.tolist()
        accuracy_grad.append(self.accuracy[2])

        for i, _ in enumerate(err):
            w = np.random.rand()
            if w < self.w1:
                err[i] = np.random.normal(0, accuracy_grad[i]/3)
            elif w < self.w2:
                err[i] = 2*self.accuracy_blowout_k[i]*accuracy_grad[i]*np.random.rand()-self.accuracy_blowout_k[i]*accuracy_grad[i]
            else:
                err[i] = 2*self.acuuracy_max_blowout_k[i]*accuracy_grad[i]*np.random.rand()-self.acuuracy_max_blowout_k[i]*accuracy_grad[i]
        
        msg.latitude += err[0]
        msg.longitude += err[1]
        msg.altitude += err[2]

        return [msg.latitude + err[0], msg.longitude + err[1], msg.altitude + err[2]]


class ImuNoizer():
    def __init__(self, vehicle_name, params) -> None:
        rospy.Subscriber('/airsim_node/'+vehicle_name+'/imu/Imu', Imu, self.imu_data_cb)
        self.publisher = rospy.Publisher('/'+vehicle_name+'/ins_imu_topic', Imu, queue_size=10)

        self.rate = 1/params.imu_rate

        rospy.Timer(rospy.Duration(self.rate), self.publish_imu_data)

        self.accuracy_orientation = params.imu_orientation_accuracy
        self.accuracy_angular_velocity = params.imu_angular_velocity_accuracy
        self.accuracy_linear_acceleration = params.imu_linear_acceleration_accuracy

    def imu_data_cb(self, msg) -> None:
        self.msg = msg

    def publish_imu_data(self, event) -> None:
        new_msg = self.probabilisticModel(self.msg)
        self.publisher.publish(new_msg)

    def probabilisticModel(self, msg) -> Imu:

        orient_err = [0, 0, 0]
        angular_vel_err = [0, 0, 0]
        linear_acc_err = [0, 0, 0]

        for i in range(len(orient_err)):
            orient_err[i] = np.random.normal(0.0, self.accuracy_orientation/3)
            angular_vel_err[i] = np.random.normal(0.0, self.accuracy_angular_velocity/3)
            linear_acc_err[i] = np.random.normal(0.0, self.accuracy_linear_acceleration/3)

        rpy = self.euleFromQuaternion(msg.orientation)

        rpy = np.array(rpy) + np.array(orient_err)

        q = self.quaternionFromEuler(*rpy.tolist())

        new_msg = Imu()
        new_msg.header = msg.header

        new_msg.orientation.x = q[1]
        new_msg.orientation.y = q[2]
        new_msg.orientation.z = q[3]
        new_msg.orientation.w = q[0]

        new_msg.angular_velocity.x = msg.angular_velocity.x + angular_vel_err[0]
        new_msg.angular_velocity.y = msg.angular_velocity.y + angular_vel_err[1]
        new_msg.angular_velocity.z = msg.angular_velocity.z + angular_vel_err[2]

        new_msg.linear_acceleration.x = msg.linear_acceleration.x + linear_acc_err[0] 
        new_msg.linear_acceleration.y = msg.linear_acceleration.y + linear_acc_err[1]
        new_msg.linear_acceleration.z = msg.linear_acceleration.z + linear_acc_err[2]

        return new_msg

    def euleFromQuaternion(self, quaternion) -> list:

        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]
    
    def quaternionFromEuler(self, roll, pitch, yaw) -> list:
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

if __name__=='__main__':    
    rospy.init_node('data_noizer_node')
    vehicle_name = ""

    while(vehicle_name == ""):
        vehicle_name = rospy.get_param('/vehicle_name', vehicle_name)
        rospy.loginfo("Wait vechical name")

    geo_param = GeodeticParam()
    params = ParametersLoader()
    
    lidar_n = LidarNoizer(vehicle_name, params)
    aion_n = AionNoizer(vehicle_name, geo_param, params)
    gps_n = GpsNoizer(vehicle_name, geo_param, params)
    imu_n = ImuNoizer(vehicle_name, params)

    rospy.spin()