#!/usr/bin/python3

import message_filters
import rospy
import tf
from geometry_msgs.msg import Quaternion
from operator import add
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer

def callback(body, cam):
	# Solve all of perception here...
	imu = Imu()
	imu.header = cam.header

	imu.orientation = Quaternion(*tf.transformations.quaternion_multiply([cam.orientation.x, cam.orientation.y, cam.orientation.z, cam.orientation.w],[body.orientation.x, body.orientation.y, body.orientation.z, -body.orientation.w]))
	imu.orientation.z = -imu.orientation.z
	imu.orientation.w = -imu.orientation.w
	imu.orientation_covariance = map(add, cam.orientation_covariance, body.orientation_covariance)

	imu.linear_acceleration = cam.linear_acceleration
	imu.linear_acceleration_covariance = cam.linear_acceleration_covariance

	imu.angular_velocity.x = cam.angular_velocity.x - body.angular_velocity.x
	imu.angular_velocity.y = cam.angular_velocity.y - body.angular_velocity.y
	imu.angular_velocity.z = cam.angular_velocity.z - body.angular_velocity.z
	# imu.angular_velocity_covariance = map(add, cam.angular_velocity_covariance, body.angular_velocity_covariance)
	imu.angular_velocity_covariance = [0.01,0,0,0,0.01,0,0,0,0.01]
	cam_pub.publish(imu)

def callback_reloc(body, cam):
    odom = PoseWithCovarianceStamped()
    odom = body
    loc_pub.publish(odom)

body_sub = message_filters.Subscriber('/imu_body/data', Imu)
cam_sub = message_filters.Subscriber('/imu_cam/data', Imu)

rob_local = message_filters.Subscriber('/rtabmap/localization_pose', PoseWithCovarianceStamped)
ekf_cam = message_filters.Subscriber('/odometry/filtered', Odometry)

cam_pub = rospy.Publisher('/imu_cam_filtered/data', Imu, queue_size=10)
loc_pub = rospy.Publisher('/robot_reloc_filtered', PoseWithCovarianceStamped, queue_size=10)

rospy.init_node('imu_cam_filtered', anonymous=True)

ts = message_filters.TimeSynchronizer([body_sub, cam_sub], 100)
ts.registerCallback(callback)

ts2 = ApproximateTimeSynchronizer([rob_local, ekf_cam], 10, 0.2)
ts2.registerCallback(callback_reloc)

rospy.spin()
