#!/usr/bin/python

import message_filters
import rospy
import tf
from geometry_msgs.msg import Quaternion
from librepilot.msg import gyro_bias
from operator import add
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer

def callback(body, cam):
	# Solve all of perception here...
	global first, estimate_x, estimate_y, estimate_z
	if first < 1:
		estimate_x +=  cam.angular_velocity.x - body.angular_velocity.x
		estimate_y +=  cam.angular_velocity.y + body.angular_velocity.z
		estimate_z +=  cam.angular_velocity.z + body.angular_velocity.y
		first += 1
	else:
		imu = Imu()
		imu.header = cam.header
		imu.header.frame_id = "cameraholder_link"

		imu.orientation = Quaternion(*tf.transformations.quaternion_multiply([cam.orientation.x, cam.orientation.y, cam.orientation.z, cam.orientation.w],[body.orientation.x, body.orientation.y, body.orientation.z, -body.orientation.w]))
		imu.orientation.z = -imu.orientation.z
		imu.orientation.w = -imu.orientation.w
		imu.orientation_covariance = map(add, cam.orientation_covariance, body.orientation_covariance)

		imu.linear_acceleration = cam.linear_acceleration
		imu.linear_acceleration_covariance = cam.linear_acceleration_covariance
#		imu.angular_velocity.x = cam.angular_velocity.x - body.angular_velocity.x - estimate_x # + (1 if estimate_x > 0 else -1) * estimate_x/first
#		imu.angular_velocity.y = cam.angular_velocity.z + body.angular_velocity.y - estimate_y # + (-1 if estimate_y > 0 else 1) * estimate_y/first
		imu.angular_velocity.z = cam.angular_velocity.y + body.angular_velocity.z #+ estimate_y # + (1 if estimate_z > 0 else -1) * estimate_z/first

		print(estimate_y)
		print(str(imu.angular_velocity.z) + '\n')
		# imu.angular_velocity_covariance = map(add, cam.angular_velocity_covariance, body.angular_velocity_covariance)
		imu.angular_velocity_covariance = [0.01,0,0,0,0.01,0,0,0,0.01]
		cam_pub.publish(imu)

def callback_reloc(body, cam):
    odom = PoseWithCovarianceStamped()
    odom = body
    loc_pub.publish(odom)

def callback_bias(msg):
	global estimate_x
	global estimate_y
	global estimate_z
	estimate_x = msg.bias.x * np.pi / 180
	estimate_y = msg.bias.y * np.pi / 180
	estimate_z = msg.bias.z * np.pi / 180


first = 0
estimate_z = 0
estimate_x = 0
estimate_y = 0

bias_sub = rospy.Subscriber('/machine_/gyro_bias', gyro_bias, callback_bias)

body_sub = message_filters.Subscriber('/imu_body/data', Imu)
cam_sub = message_filters.Subscriber('/camera/imu', Imu)

rob_local = message_filters.Subscriber('/rtabmap/localization_pose', PoseWithCovarianceStamped)
ekf_cam = message_filters.Subscriber('/odometry/filtered', Odometry)

cam_pub = rospy.Publisher('/imu_cam_filtered/data', Imu, queue_size=10)
loc_pub = rospy.Publisher('/robot_reloc_filtered', PoseWithCovarianceStamped, queue_size=10)

rospy.init_node('imu_cam_filtered', anonymous=True)

ts = ApproximateTimeSynchronizer([body_sub, cam_sub], 1, 0.2)
ts.registerCallback(callback)

ts2 = ApproximateTimeSynchronizer([rob_local, ekf_cam], 10, 0.2)
ts2.registerCallback(callback_reloc)

rospy.spin()
