#!/usr/bin/env python

import rospy
import numpy as np
import tf
from std_msgs.msg import Float32, Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

phase = 0
rot_speed = 0


def get_phase(data):
    #print(data.data)
     global phase 
     phase = data.data

def get_rot_speed(data):
    # print(data.data)
     global rot_speed 
     rot_speed = data.data

def get_time(data):
    global arduino_time
    arduino_time = data.data

def encoder_node():
   pub = rospy.Publisher('/encoder/odometry', Odometry, queue_size=10)
   rospy.init_node('encoder_node', anonymous=True)
   
   rate = rospy.Rate(10) # 10hz
   while not rospy.is_shutdown():

##### Get phase and rot_speed
      rospy.Subscriber('/encoder/speed', Float32, get_rot_speed, queue_size=100)
      rospy.Subscriber('/encoder/phase',Float32,get_phase,queue_size=100)
      rospy.Subscriber('/encoder/time',Time,get_time,queue_size=100)
      #rospy.Subscriber('/encoder/rot_speed', Float32, get_rot_speed, queue_size=100)

###### Odom #######
      msg = Odometry()
      msg.header.stamp = arduino_time # Get timestamp recorded at arduino
      msg.header.frame_id = 'pantilt_link'
      #set position
      msg.pose.pose.position.x = 0 # do for y and z      
      msg.pose.pose.position.y = 0 # do for y and z      
      msg.pose.pose.position.z = 0 # do for y and z 
      #Define quaternion rotation around z axis
      msg.pose.pose.orientation.x = np.cos(phase/2) #  x [quaternion]
      msg.pose.pose.orientation.y = 0 #  y [quaternion]
      msg.pose.pose.orientation.z = 0 #  z [quaternion]
      msg.pose.pose.orientation.w = np.sin(phase/2) #  w [quaternion] 
      
      cov = [1e-3,0,0,0,0,0,
             0,1e-3,0,0,0,0,
             0,0,1e-3,0,0,0,
             0,0,0,1e-3,0,0,
             0,0,0,0,1e-3,0,
             0,0,0,0,0,1e-3]

      msg.pose.covariance = cov 

      #set velocity
      msg.child_frame_id = 'cameraholder_link'
      msg.twist.twist.linear.x = 0;
      msg.twist.twist.linear.y = 0;
      msg.twist.twist.angular.z = rot_speed 
      msg.twist.covariance = cov 

      
      #rospy.loginfo(msg)
      pub.publish(msg)
      rate.sleep()

if __name__ == '__main__':
   try:
      encoder_node()
   except rospy.ROSInterruptException:
      pass
