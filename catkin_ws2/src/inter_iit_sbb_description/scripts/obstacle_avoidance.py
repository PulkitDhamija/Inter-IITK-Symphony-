#!/usr/bin/env python

import numpy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu , NavSatFix, JointState
from geometry_msgs.msg import Quaternion , Vector3Stamped
import rospy
import tf
from scripts import PID

class ObstacleAvoidance:
    def __init__(self):
        
        rospy.init_node('obstacle_avoidance')
        
        self.angle = [0.0, 0.0, 0.0]
        self.time = [0.0, 0.0]
        self.range = [0.0, 0.0, 0.0]
        self.intensity = 0.0

        rospy.Subscriber('/sbb/distance_sensor/front', LaserScan, self.l_callback)

        def l_callback(self, msg):

            self.angle[0] = msg.angle_min
            self.angle[1] = msg.angle_max
            self.angle[0] = msg.angle_increment
            self.time[0] = msg.time_increment
            self.time[1] = msg.scan_time
            self.range[0] = msg.range_min
            self.range[1] = msg.range_max
            self.range[2] = msg.ranges
            self.intensity = msg.intensities