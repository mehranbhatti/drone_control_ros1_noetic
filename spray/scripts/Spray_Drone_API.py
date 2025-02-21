#!/usr/bin/python3

import math
import rospy

from Drone_API import MAVROS_Drone


class SprayDrone(MAVROS_Drone):
    def __init__(self, name='drone1', *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self.ns = name
        self.data.header.name = name
        self.data.header.id = 1
        
        rospy.init_node('Spray_Drone')
        
        self.initialize()
        
    def initialize(self):
        # Initialize leader's ROS subscribers and publishers
        self.init_subscribers()
        self.init_publishers()


    def wait_for_GPS_Fix(self):
        while True:
            if self.data.global_position.gps_fix == 0:
                print("Leader GPS Fix acquired!")
                break
