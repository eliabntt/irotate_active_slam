/*
 * OdometryROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "OdometryROS.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

OdometryROS::OdometryROS()
{
	odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1, true);

	reset_odometry_server_ = nh_.advertiseService("reset_odometry",
			&OdometryROS::resetOdometryCallback, this);
}

OdometryROS::~OdometryROS()
{
	odometry_pub_.shutdown();
	reset_odometry_server_.shutdown();
}

void OdometryROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}
void OdometryROS::readingsEvent(double x, double y, double phi,
		float vx, float vy, float omega, unsigned int sequence )
{
	geometry_msgs::Quaternion phi_quat = tf::createQuaternionMsgFromYaw( phi );

	// Construct messages
	odometry_msg_.header.seq = sequence;
	odometry_msg_.header.frame_id = "odom";
	odometry_msg_.header.stamp = stamp_;
	odometry_msg_.child_frame_id = "base_link";
	odometry_msg_.pose.pose.position.x = x ;
	odometry_msg_.pose.pose.position.y = y ;
	odometry_msg_.pose.pose.position.z = 0.0;
	odometry_msg_.pose.pose.orientation = phi_quat;
	odometry_msg_.twist.twist.linear.x = vx;
	odometry_msg_.twist.twist.linear.y = vy;
	odometry_msg_.twist.twist.linear.z = 0.0;
	odometry_msg_.twist.twist.angular.x = 0.0;
	odometry_msg_.twist.twist.angular.y = 0.0;
	odometry_msg_.twist.twist.angular.z = omega;

	odometry_transform_.header.frame_id = "odom";
	odometry_transform_.header.stamp = odometry_msg_.header.stamp;
	odometry_transform_.child_frame_id = "base_link";
	odometry_transform_.transform.translation.x = x;
	odometry_transform_.transform.translation.y = y;
	odometry_transform_.transform.translation.z = 0.0;
	odometry_transform_.transform.rotation = phi_quat;

	odometry_transform_broadcaster_.sendTransform( odometry_transform_ );

	// Publish the msg
	odometry_pub_.publish( odometry_msg_ );
}

bool OdometryROS::resetOdometryCallback(
		robotino_msgs::ResetOdometry::Request &req,
		robotino_msgs::ResetOdometry::Response &res)
{
	set( req.x, req.y, req.phi, true );

	return true;
}
