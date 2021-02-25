/*
 * OdometryROS.h
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ODOMETRYROS_H_
#define ODOMETRYROS_H_

#include "rec/robotino/api2/Odometry.h"
#include "robotino_msgs/ResetOdometry.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

class OdometryROS: public rec::robotino::api2::Odometry
{
public:
	OdometryROS();
	~OdometryROS();

	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	ros::Publisher odometry_pub_;

	ros::ServiceServer reset_odometry_server_;

	nav_msgs::Odometry odometry_msg_;
	geometry_msgs::TransformStamped odometry_transform_;

	tf::TransformBroadcaster odometry_transform_broadcaster_;

	ros::Time stamp_;


	void readingsEvent(double x, double y, double phi,
			float vx, float vy, float omega, unsigned int sequence );
	bool resetOdometryCallback(
			robotino_msgs::ResetOdometry::Request &req,
			robotino_msgs::ResetOdometry::Response &res);
};

#endif /* ODOMETRYROS_H_ */
