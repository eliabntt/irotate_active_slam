#!/usr/bin/env python

import rospy
import numpy as np
import tf
from std_msgs.msg import Float32, Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class Encoder_Node(object):
	def __init__(self):
		self.phase_sub = rospy.Subscriber('/encoder/phase',Float32,self.get_phase_cb)
		self.phase = Float32()

	def get_phase_cb(self,data):
		self.phase = data
		print(self.phase)

if __name__ == "__main__":
	rospy.init_node('encoder_node',anonomyous=True)
	encoder_node = Encoder_Node()
	rospy.spin()