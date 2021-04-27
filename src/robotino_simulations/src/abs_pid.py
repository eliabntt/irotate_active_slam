#!/usr/bin/python

import message_filters
import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

is_pos = True

def odom_repub(msg):
 i = Float64()
 i.data = abs(msg.twist.twist.angular.z)
 odom_pub.publish(i)

def setpoint_repub(msg):
 global is_pos
 if msg.data > 0:
  is_pos = True
 else:
  is_pos = False
 if msg.data == 0:
  i = Float64()
  i.data = 52
  effort_pub.publish(i)
 i = Float64()
 i.data = abs(msg.data)
 setpoint_pub.publish(i)

def control_effort_repub(msg):
 global is_pos
 i = Float64()
 if is_pos:
  i.data = msg.data
 else:
  i.data = -msg.data
 effort_pub.publish(i)


rospy.init_node("my_abs_handler")
sub_odom = rospy.Subscriber("/camera/odometry/filtered", Odometry, odom_repub)
sub_setpoint = rospy.Subscriber("/encoder/setpoint", Float64, setpoint_repub)
sub_effort = rospy.Subscriber("/encoder/control_effort_abs", Float64, control_effort_repub)


setpoint_pub = rospy.Publisher("/encoder/setpoint_abs", Float64)
effort_pub = rospy.Publisher("/encoder/control_effort", Float64)
odom_pub = rospy.Publisher("/encoder/rot_speed_filtered", Float64)
rospy.spin()
