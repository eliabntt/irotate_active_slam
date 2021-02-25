/*
 * OmniDriveROS.h
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef OMNIDRIVEROS_H_
#define OMNIDRIVEROS_H_

#include "rec/robotino/api2/OmniDrive.h"

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

class OmniDriveROS: public rec::robotino::api2::OmniDrive
{
public:
	OmniDriveROS();
	~OmniDriveROS();

private:
	ros::NodeHandle nh_;

	ros::Subscriber cmd_vel_sub_;

	double max_linear_vel_;
	double min_linear_vel_;
	double max_angular_vel_;
	double min_angular_vel_;

	void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);

public:
	void setMaxMin( double max_linear_vel, double min_linear_vel,
				double max_angular_vel, double min_angular_vel );
};

#endif /* OMNIDRIVEROS_H_ */
