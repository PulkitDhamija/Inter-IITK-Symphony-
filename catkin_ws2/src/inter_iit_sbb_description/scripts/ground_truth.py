#!/usr/bin/env python

import numpy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu , NavSatFix, JointState
from geometry_msgs.msg import Quaternion , Vector3Stamped , PoseStamped
import rospy
import tf
from scripts import PID
from rosgraph_msgs.msg import Clock

class GroundTruthNode:
    def __init__(self):

        rospy.init_node('ground_truth')
        
        self.sbb_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
        self.sbb_orientation_euler = [0.0, 0.0, 0.0]
        self.sbb_position_gcoordinates = [0.0, 0.0, 0.0]
        self.sbb_position_coordinates = [0.0, 0.0, 0.0]
        self.sbb_angular_velocity = [0.0, 0.0, 0.0]
        self.sbb_linear_acceleration = [0.0, 0.0, 0.0]
        self.sbb_linear_velocity = [0.0, 0.0, 0.0]

        self.initialized = True
        self.msg = Clock()


        rospy.Subscriber('/sbb/imu', Imu, self.imu_callback)
        rospy.Subscriber('/sbb/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/sbb/gps/vel', Vector3Stamped, self.gps_vel_callback)
        self.time_sub = rospy.Subscriber('/clock', Clock, self.time_callback)
        self.data_pub = rospy.Publisher('sbb/ground_truth', PoseStamped, queue_size=10)

    def time_callback(self, msg):
        self.time = msg
        
    
    def gps_callback(self, msg):
		
	    self.sbb_position_gcoordinates[0] = msg.latitude
	    self.sbb_position_gcoordinates[1] = msg.longitude
	    self.sbb_position_gcoordinates[2] = msg.altitude

    def gps_vel_callback(self, msg):
        
        self.sbb_linear_velocity[0] = msg.vector.x
        self.sbb_linear_velocity[1] = msg.vector.y
        self.sbb_linear_velocity[2] = msg.vector.z

    def imu_callback(self, msg):

	    self.sbb_orientation_quaternion[0] = msg.orientation.x
	    self.sbb_orientation_quaternion[1] = msg.orientation.y
	    self.sbb_orientation_quaternion[2] = msg.orientation.z
	    self.sbb_orientation_quaternion[3] = msg.orientation.w
	    self.sbb_angular_velocity[0] = msg.angular_velocity.x
	    self.sbb_angular_velocity[1] = msg.angular_velocity.y
	    self.sbb_angular_velocity[2] = msg.angular_velocity.z
	    self.sbb_linear_acceleration[0] = msg.linear_acceleration.x
	    self.sbb_linear_acceleration[1] = msg.linear_acceleration.y
	    self.sbb_linear_acceleration[2] = msg.linear_acceleration.z
    