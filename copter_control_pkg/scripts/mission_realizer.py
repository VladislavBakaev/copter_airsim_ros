#!/usr/bin/python3

import rospy
import rospkg
import os
import re
from airsim_ros_pkgs.srv import SetGPSPosition, Takeoff, SetLocalPosition
from std_srvs.srv import SetBool

class MissionRealizer():
    def __init__(self, mission_file, vehicle_name) -> None:
        self.mission_file = mission_file
        self.vehicle_name = vehicle_name
        self.mission_point = {}
        self.set_gps_mission = rospy.ServiceProxy('/airsim_node/gps_goal', SetGPSPosition)
        self.takeOff = rospy.ServiceProxy('/airsim_node/drone/takeoff', Takeoff)
        self.local_move = rospy.ServiceProxy('/airsim_node/local_position_goal', SetLocalPosition)
        rospy.wait_for_service('/airsim_node/gps_goal')
        self.loadMission()
        self.realize()

    def loadMission(self):
        with open(self.mission_file, 'r') as txt_file:
            data = txt_file.read()
            data = re.findall(r'\[(\d+)\]\s(.*)\s(.*)\s(.*)\s(.*)\s(.*)\s(.*)', data)
            for point in data:
                self.mission_point[point[0]] = {}
                for i in range(1, len(point)):
                    param = point[i].split('=')
                    self.mission_point[point[0]].update({param[0]:float(param[1])})
            
            self.iter_seq = sorted(self.mission_point.keys())
    
    def realize(self):
        self.takeOff(True)
        self.local_move(0.0, 0.0, -5.0, 0.0, self.vehicle_name)
        for i in range(len(self.mission_point)):
            while (True):
                try:
                    point = self.mission_point[self.iter_seq[i]]
                    lat = point['Lat']
                    lon = point['Lon']
                    alt = point['Alt']
                    yaw = point['Yaw']
                    self.set_gps_mission(lat, lon, alt, yaw, self.vehicle_name)
                    break
                except:
                    rospy.loginfo('Wait...')
                    rospy.sleep(0.5)
                    continue


if __name__=="__main__":
    rospy.init_node('mission_realizer_node')

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('copter_control_pkg')
    file_path = os.path.join(pkg_path, 'mission', 'mission.txt')

    vehicle_name = ""

    while(vehicle_name == ""):
        vehicle_name = rospy.get_param('/vehicle_name', vehicle_name)
        rospy.loginfo("Wait vechical name")

    mission_realizer = MissionRealizer(file_path, vehicle_name)