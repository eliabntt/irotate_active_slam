/*
 * BumperROS.h
 *
 *  Created on: 06.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef BUMPERROS_H_
#define BUMPERROS_H_

#include "rec/robotino/api2/Bumper.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>

class BumperROS: public rec::robotino::api2::Bumper
{
public:
	BumperROS();
	~BumperROS();

private:
	ros::NodeHandle nh_;

	ros::Publisher bumper_pub_;

	std_msgs::Bool bumper_msg_;

	void bumperEvent(bool hasContact);
};

#endif /* BUMPERROS_H_ */
