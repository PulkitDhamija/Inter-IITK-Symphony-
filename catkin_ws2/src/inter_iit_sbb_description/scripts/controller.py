#!/usr/bin/env python

import numpy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu , NavSatFix, JointState
from geometry_msgs.msg import Quaternion , Vector3Stamped , PoseStamped
import rospy
import tf
from scripts import PID

class sbb():
	def __init__(self):

		rospy.init_node('controller')
		
		self.flywheel = [0.0, 0.0, 0.0]
		self.drive_wheel = [0.0, 0.0, 0.0]
		self.handle = [0.0, 0.0, 0.0]
		self.stand1 = [0.0, 0.0, 0.0]
		self.stand2 = [0.0, 0.0, 0.0]
		self.angle = [0.0, 0.0, 0.0]
		self.time = [0.0, 0.0]
		self.range = [0.0, 0.0, 0.0]
		self.intensity = 0.0

		self.bot_position = [0.0, 0.0, 0.0]
		self.bot_orientation = [0.0, 0.0, 0.0, 0.0]

		self.sample_rate = 50.0  # in Hz

		self.i = 0.0
		self.curr_angle = 0.0
		self.prev_angle = 0.0
		self.reset = 0

		self.data_cmd = Float32()
		self.data_cmd.data = 0.0

		self.pid_rot = PID.PIDRegulator(1, 0, 0, 1)
		self.pid_pos = PID.PIDRegulator(1, 0, 0, 1)

		self.data_pub1 = rospy.Publisher('/flywheel/command', Float32, queue_size=1)
		self.data_pub2 = rospy.Publisher('/drive_wheel/command', Float32, queue_size=1)
		self.data_pub3 = rospy.Publisher('/handle/command', Float32, queue_size=1)
		self.data_pub4 = rospy.Publisher('/stand1/command', Float32, queue_size=1)
		self.data_pub5 = rospy.Publisher('/stand2/command', Float32, queue_size=1)

		rospy.Subscriber('/sbb/imu', Imu, self.imu_callback)
		rospy.Subscriber('/sbb/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/sbb/gps/vel', Vector3Stamped, self.gps_vel_callback)
		rospy.Subscriber('/flywheel/joint_state', JointState, self.fw_callback)
		rospy.Subscriber('/drive_wheel/joint_state', JointState, self.dw_callback)
		rospy.Subscriber('/handle/joint_state', JointState, self.hd_callback)
		rospy.Subscriber('/stand1/joint_state', JointState, self.std1_callback)
		rospy.Subscriber('/stand2/joint_state', JointState, self.std2_callback)
		rospy.Subscriber('/sbb/distance_sensor/front', LaserScan, self.l_callback)
		rospy.Subscriber('/ground_truth1', PoseStamped, self.gt_callback)

	def fw_callback(self, msg):

		self.flywheel[0] = msg.position
		self.flywheel[1] = msg.velocity
		self.flywheel[2] = msg.effort

	def dw_callback(self, msg):

		self.drive_wheel[0] = msg.position
		self.drive_wheel[1] = msg.velocity
		self.drive_wheel[2] = msg.effort

	def hd_callback(self, msg):

		self.handle[0] = msg.position
		self.handle[1] = msg.velocity
		self.handle[2] = msg.effort

	def std1_callback(self, msg):

		self.stand1[0] = msg.position
		self.stand1[1] = msg.velocity
		self.stand1[2] = msg.effort

	def std2_callback(self, msg):

		self.stand2[0] = msg.position
		self.stand2[1] = msg.velocity
		self.stand2[2] = msg.effort


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
 
	def gt_callback(self, msg):
		self.bot_position[0] = msg.pose.position.x
		self.bot_position[1] = msg.pose.position.y
		self.bot_position[2] = msg.pose.position.z
		self.bot_orientation[0] = msg.pose.orientation.x
		self.bot_orientation[1] = msg.pose.orientation.y
		self.bot_orientation[2] = msg.pose.orientation.z
		self.bot_orientation[3] = msg.pose.orienatation.w


	
 # conversion of global coordinates to map coordinates
 # check topic link states (but it is a default topic of link staes)
 # desired pose to be converted into map ccordinates and stored in variables and orientation too
 # check the sensors data to see how the sensor works (its initial readings)
 # traction meaning
 # Check how command should be given in different scenerios
 # Check what position velocity effort means in different links
 # Check how to use lidar data for obstacle avoidance (use of if statement)
 # Optimisation

	def pid(self):
	    (self.sbb_orientation_euler[1], self.sbb_orientation_euler[0], self.sbb_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.sbb_orientation_quaternion[0], self.sbb_orientation_quaternion[1], self.sbb_orientation_quaternion[2], self.sbb_orientation_quaternion[3]])
	    print(self.sbb_orientation_euler[1])
	    self.curr_angle = self.sbb_orientation_euler[1]

	    if self.curr_angle > 0:

	    	if self.reset == 0:
	    		self.i = 0

	    	self.i-= 5*self.curr_angle
	    	self.data_cmd.data = self.i
	    	self.reset = 1

	    if self.curr_angle < 0:
	    	if self.reset == 1:
	    		self.i = 0
	    	self.i-= 5*self.curr_angle
	    	self.data_cmd.data = self.i
	    	self.reset = 0

	    self.prev_angle = self.curr_angle
	    #print(self.data_cmd.data)

	    self.data_pub.publish(self.data_cmd)

	    




if __name__ == '__main__':

    sbb = sbb()
    r = rospy.Rate(sbb.sample_rate)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
     
        try:    
            sbb.pid()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException: pass