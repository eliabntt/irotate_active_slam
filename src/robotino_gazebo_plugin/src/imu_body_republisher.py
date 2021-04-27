#!/usr/bin/python

import message_filters
import rospy
import tf
from geometry_msgs.msg import Quaternion
from operator import add
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer

from librepilot.msg import gyro_bias

def raw_imu_callback(body):
	global first, estimate_x, estimate_y, estimate_z
	if first < 1:
		estimate_x += body.angular_velocity.x
		estimate_y += body.angular_velocity.x
		estimate_z += body.angular_velocity.x
		first += 1
	else:
		imu = body
		imu.header.frame_id = "imu_link"
		imu.angular_velocity_covariance = [0.01,0,0,0,0.01,0,0,0,0.01]
		imu.angular_velocity.x += estimate_x #(1 if estimate_x > 0 else -1) * estimate_x/first
		imu.angular_velocity.y += estimate_z #(-1 if estimate_y > 0 else 1) * estimate_y/first
		imu.angular_velocity.z += estimate_y #(-1 if estimate_z > 0 else 1) * estimate_z/first
		body_pub.publish(imu)

def callback_bias(msg):
	global estimate_x
	global estimate_y
	global estimate_z
	estimate_x = msg.bias.x * np.pi / 180
	estimate_y = msg.bias.y * np.pi / 180
	estimate_z = msg.bias.z * np.pi / 180

rospy.init_node('imu_body_republisher', anonymous=True)
first = 0
estimate_z = 0
estimate_x = 0
estimate_y = 0
bias_sub = rospy.Subscriber('/machine_/gyro_bias', gyro_bias, callback_bias)
body_sub_raw = rospy.Subscriber('/machine_/Imu', Imu, raw_imu_callback, queue_size=1)
body_pub = rospy.Publisher('/imu_body/data', Imu, queue_size=1)


rospy.spin()