#!/usr/bin/env python

import rospy
import numpy as np
import tf
from std_msgs.msg import Float32, Float64, Time, Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

phase = 0
rot_speed = 0
arduino_time = rospy.Time(0)
interval_ms = 1
is_ard = False

def get_header(data):
    global arduino_time
    global interval_ms
    global is_ard
    is_ard = True
    arduino_time = data.stamp
    if data.frame_id == "positive":
       interval_ms = data.seq
    else:
       interval_ms = -data.seq
    

def encoder_node():
   pub_odom = rospy.Publisher('/encoder/odometry', Odometry, queue_size=10)
   pub_rot_speed = rospy.Publisher('/encoder/rot_speed',Float64,queue_size=10)
   pub_rot_speed_neg = rospy.Publisher('/encoder/rot_speed_neg',Float64,queue_size=10)
   rospy.init_node('encoder_node', anonymous=True)
   
   rate = rospy.Rate(100) 
   global arduino_time
   global is_ard
   global interval_ms

   while not rospy.is_shutdown():

##### read header
      rospy.Subscriber('/encoder/header',Header,get_header,queue_size=1)
      if is_ard:
	      rot_speed = (3.141592/180)*0.36*1000000/interval_ms
      else:
          rot_speed = 0
      #print(arduino_time)

###### Odom #######
      msg_odom = Odometry()
      if not is_ard:
         arduino_time = rospy.Time().now()

      is_ard = False
      msg_odom.header.stamp = arduino_time # Get timestamp recorded at arduino
      msg_odom.header.frame_id = 'pantilt_link'
      #set position
      msg_odom.pose.pose.position.x = 0 # do for y and z      
      msg_odom.pose.pose.position.y = 0 # do for y and z      
      msg_odom.pose.pose.position.z = 0 # do for y and z 
      #Define quaternion rotation around z axis
      #msgOdom.pose.pose.orientation.x = np.cos(phase/2) #  x [quaternion]
      msg_odom.pose.pose.orientation.x = 0 
      msg_odom.pose.pose.orientation.y = 0 #  y [quaternion]
      msg_odom.pose.pose.orientation.z = 0 #  z [quaternion]
    #  msgOdom.pose.pose.orientation.w = np.sin(phase/2) #  w [quaternion] 
      msg_odom.pose.pose.orientation.w = 0
      
      cov = [1e-3,0,0,0,0,0,
             0,1e-3,0,0,0,0,
             0,0,1e-3,0,0,0,
             0,0,0,1e-3,0,0,
             0,0,0,0,1e-3,0,
             0,0,0,0,0,1e-3]

      msg_odom.pose.covariance = cov 

      #set velocity
      msg_odom.child_frame_id = 'cameraholder_link'
      msg_odom.twist.twist.linear.x = 0;
      msg_odom.twist.twist.linear.y = 0;

      msg_odom.twist.twist.angular.z = rot_speed 
      msg_odom.twist.covariance = cov 

      
      #rospy.loginfo(msg)
      pub_odom.publish(msg_odom)
      

##### Convert pid topics
      msg_rot_speed = Float64()
      msg_rot_speed.data = rot_speed
      pub_rot_speed.publish(msg_rot_speed)
      
      msg_rot_speed_neg = Float64()
      msg_rot_speed_neg.data = -rot_speed
      pub_rot_speed_neg.publish(msg_rot_speed_neg)
     

      rate.sleep()
if __name__ == '__main__':
   try:
      encoder_node()
   except rospy.ROSInterruptException:
      pass
