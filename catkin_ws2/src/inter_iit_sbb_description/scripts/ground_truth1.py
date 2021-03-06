#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import PoseStamped
import numpy
from rospy.numpy_msg import numpy_msg
from gazebo_msgs.msg import LinkStates
from rosgraph_msgs.msg import Clock

class GroundTruthNode:
    def __init__(self):
        
        self.link_name = 'triton::triton/base_link'

        self.pos = numpy.zeros(3)
        self.quat = numpy.array([0, 0, 0, 1])
        self.initialized = True
        self.msg = Clock()
        self.gaz_sub = rospy.Subscriber('/gazebo/link_states', numpy_msg(LinkStates), self.link_states_callback)
        self.time_sub = rospy.Subscriber('/clock', Clock, self.time_callback)
        self.odm_pub = rospy.Publisher('ground_truth1', PoseStamped, queue_size=10)

    def time_callback(self, msg):
        self.time = msg
        # print(self.time.clock.secs)
        # print(msg.clock.secs)

    def link_states_callback(self, msg):
        index = msg.name.index(self.link_name)
        odm = PoseStamped()
        odm.pose = msg.pose[index]
        odm.header.stamp = self.time.clock
        self.odm_pub.publish(odm)
        # print(odm)

if __name__ == '__main__':
    print('starting Ground Truth 1')
    rospy.init_node('ground_truth1')

    try:
        node = GroundTruthNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')

