#!/usr/bin/env python3
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, Int16
from nav_msgs.msg import OccupancyGrid
from rtabmap_ros.msg import MapData
from rtabmap_ros.msg import Info
import numpy as np
from matplotlib import pyplot as plt
import rospy
from math import log
from nav_msgs.msg import Odometry


def grid(msg):
    global previous
    global previous_entropy
    global f_map
    tot_entropy = 0
    marginal_entropy = 0
    data = np.array(msg.data)
    res = np.where(data != -1)
    for i in previous:
        if data[i] != -1:
            if data[i] == 0:
                prob = 0.00001
            elif data[i] == 100:
                prob = 0.99999999999999
            else:
                prob = (data[i] / 100.0)
            try:
                marginal_entropy += -(prob * log(prob, 2) + (1 - prob) * log(1 - prob, 2))
            except ValueError:
                print("Error with prob " + str(prob))

    marginal_entropy -= previous_entropy
    previous = res[0]
    tot_area = len(res[0]) * msg.info.resolution * msg.info.resolution
    for i in res[0]:
        if data[i] != -1:
            if data[i] == 0:
                prob = 0.00001
            elif data[i] == 100:
                prob = 0.99999999999999
            else:
                prob = (data[i] / 100.0)
            tot_entropy += -(prob * log(prob, 2) + (1 - prob) * log(1 - prob, 2))
    previous_entropy = tot_entropy
    mme.publish(marginal_entropy)
    me.publish(tot_entropy)
    ta.publish(tot_area)
    f_map['total_entropy'].append(tot_entropy)
    f_map['area'].append(tot_area)
    f_map['marginal'].append(marginal_entropy)
    f_map['time'].append(msg.header.stamp)
    np.save('map_data.npy', f_map)


def features(msg):
    global glc
    global llc
    global f_feat
    if msg.loopClosureId != 0:
        glc += 1
    if msg.proximityDetectionId != 0:
        llc += 1
    tglc.publish(glc)
    tllc.publish(llc)
    f_feat['local'].append(llc)
    f_feat['global'].append(glc)
    f_feat['total'].append(llc + glc)
    f_feat['time'].append(msg.header.stamp)
    cnt = 0
    for i in msg.statsKeys:
        if "Keypoint/Current_frame/words" in i:
            tf.publish(msg.statsValues[cnt])
            f_feat['feat'].append(msg.statsValues[cnt])
        cnt += 1
    np.save('feat.npy', f_feat)


def wheels(msg):
    global f_w
    f_w['time'].append(msg.header.stamp)
    f_w['wheel1'].append(msg.twist.linear.x)
    f_w['wheel2'].append(msg.twist.linear.y)
    f_w['wheel3'].append(msg.twist.linear.z)
    f_w['tot_wheel'].append(msg.twist.linear.z + msg.twist.linear.x + msg.twist.linear.y)
    np.save('wheels.npy', f_w)


def cam(msg):
    global f_c
    f_c['time'].append(msg.header.stamp)
    f_c['cam'].append(msg.twist.linear.x)
    np.save('camera.npy', f_c)

def odom(msg):
    global o_c
    o_c['time'].append(msg.header.stamp)
    o_c['cov'].append(msg.pose.covariance)
    o_c['pose'].append(msg.pose.pose)
    np.save('odom.npy',o_c)

    global wheel_from_odom
    # yaw = tf.transformation.euler_from_quaternion(msg.pose.pose.orientation)[2]

    R_WHEEL = 0.04
    R_BASE = 0.17

    v0 = -msg.twist.twist.linear.x * np.sin(np.pi/3) + msg.twist.twist.linear.y * np.cos(np.pi/3) + msg.twist.twist.angular.z * R_BASE
    v1 = -msg.twist.twist.linear.y + msg.twist.twist.angular.z * R_BASE
    v2 = +msg.twist.twist.linear.x * np.sin(np.pi/3) + msg.twist.twist.linear.y * np.cos(np.pi/3) + msg.twist.twist.angular.z * R_BASE

    w0 = abs(v0 / R_WHEEL)
    w1 = abs(v1 / R_WHEEL)
    w2 = abs(v2 / R_WHEEL)

    if len(wheel_from_odom['time']) > 0:
        wheel_from_odom['time'].append(msg.header.stamp)
        wheel_from_odom['wheel1'].append(wheel_from_odom['wheel1'][-1] + w0 * (msg.header.stamp.to_sec() - wheel_from_odom['time'][-2].to_sec()))
        wheel_from_odom['wheel2'].append(wheel_from_odom['wheel2'][-1] + w1 * (msg.header.stamp.to_sec() - wheel_from_odom['time'][-2].to_sec()))
        wheel_from_odom['wheel3'].append(wheel_from_odom['wheel3'][-1] + w2 * (msg.header.stamp.to_sec() - wheel_from_odom['time'][-2].to_sec()))
        wheel_from_odom['tot_wheel'].append(wheel_from_odom['wheel1'][-1]+wheel_from_odom['wheel2'][-1]+wheel_from_odom['wheel3'][-1])
    else:
        wheel_from_odom['time'].append(msg.header.stamp)
        wheel_from_odom['wheel1'].append(0*w0 * (msg.header.stamp.to_sec()))
        wheel_from_odom['wheel2'].append(0*w1 * (msg.header.stamp.to_sec()))
        wheel_from_odom['wheel3'].append(0*w2 * (msg.header.stamp.to_sec()))
        wheel_from_odom['tot_wheel'].append(wheel_from_odom['wheel1'][-1]+wheel_from_odom['wheel2'][-1]+wheel_from_odom['wheel3'][-1])

    np.save('wheel_from_odom.npy', wheel_from_odom)

if __name__ == '__main__':
    f_w = {'wheel1': [], 'wheel2': [], 'wheel3': [], 'tot_wheel': [], 'time': []}
    wheel_from_odom = {'wheel1': [], 'wheel2': [], 'wheel3': [], 'tot_wheel': [], 'time': []}
    f_c = {'cam': [], 'time': []}
    o_c = {'cov': [], 'pose':[], 'time': []}
    f_feat = {'local': [], 'global': [], 'total': [], 'feat': [], 'time': []}
    f_map = {'total_entropy': [], 'marginal': [], 'area': [], 'time': []}

    previous = []
    previous_entropy = 0
    llc = 0
    glc = 0
    rospy.init_node("logger")
    rospy.Subscriber("/rtabmap/grid_prob_map", OccupancyGrid, grid, queue_size=1)
    rospy.Subscriber("/rtabmap/info", Info, features, queue_size=1)
    rospy.Subscriber("/wheels", TwistStamped, wheels, queue_size=1)
    rospy.Subscriber("/camera_rot", TwistStamped, cam, queue_size=1)
    rospy.Subscriber("/odometry/filtered", Odometry, odom, queue_size=1)
    me = rospy.Publisher("/map_entropy", Float32, queue_size=1)
    mme = rospy.Publisher("/marginal_map_entropy", Float32, queue_size=1)
    ta = rospy.Publisher("/total_area", Float32, queue_size=1)
    tf = rospy.Publisher("/tot_features", Int16, queue_size=1)
    tllc = rospy.Publisher("/tot_local_lc", Int16, queue_size=1)
    tglc = rospy.Publisher("/tot_global_lc", Int16, queue_size=1)
    rospy.spin()
