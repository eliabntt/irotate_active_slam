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
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped


def callback_reloc(rob, cam):
	odom = Odometry()
	odom = rob
	odom.child_frame_id = "base_with_cam"
	odom.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_multiply([rob.pose.pose.orientation.x, rob.pose.pose.orientation.y, rob.pose.pose.orientation.z, rob.pose.pose.orientation.w],
																					[cam.pose.pose.orientation.x, cam.pose.pose.orientation.y, cam.pose.pose.orientation.z, cam.pose.pose.orientation.w]))
	odom.twist.twist.angular.z += cam.twist.twist.angular.z
	cov = list(odom.pose.covariance)
	cov[-1] += cam.pose.covariance[-1]
	odom.pose.covariance = cov

	cov = list(odom.twist.covariance)
	cov[-1] += cam.twist.covariance[-1]
	odom.twist.covariance = cov
	loc_pub.publish(odom)
	br = tf2_ros.TransformBroadcaster()
	t = TransformStamped()

	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "odom"
	t.child_frame_id = "base_with_cam"
	t.transform.translation.x = odom.pose.pose.position.x
	t.transform.translation.y = odom.pose.pose.position.y
	t.transform.translation.z = 0.0
	t.transform.rotation.x = odom.pose.pose.orientation.x
	t.transform.rotation.y = odom.pose.pose.orientation.y
	t.transform.rotation.z = odom.pose.pose.orientation.z
	t.transform.rotation.w = odom.pose.pose.orientation.w

	br.sendTransform(t)

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

ekf_cam = message_filters.Subscriber('/camera/odometry/filtered', Odometry)
ekf_rob = message_filters.Subscriber('/odometry/filtered', Odometry)
loc_pub = rospy.Publisher('/robot_cam_filtered', Odometry, queue_size=10)


ts3 = ApproximateTimeSynchronizer([ekf_rob,ekf_cam], 100, 0.2)
ts3.registerCallback(callback_reloc)

bias_sub = rospy.Subscriber('/machine_/gyro_bias', gyro_bias, callback_bias)

body_sub = message_filters.Subscriber('/imu_body/data', Imu)
cam_sub = message_filters.Subscriber('/camera/imu', Imu)

rob_local = message_filters.Subscriber('/rtabmap/localization_pose', PoseWithCovarianceStamped)
ekf_cam = message_filters.Subscriber('/odometry/filtered', Odometry)

cam_pub = rospy.Publisher('/imu_cam_filtered/data', Imu, queue_size=10)

rospy.init_node('imu_cam_filtered', anonymous=True)

ts = ApproximateTimeSynchronizer([body_sub, cam_sub], 1, 0.2)
ts.registerCallback(callback)

rospy.spin()
