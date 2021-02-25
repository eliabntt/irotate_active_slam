/*
 * RobotinoNode.h
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ROBOTINOODOMETRYNODE_H_
#define ROBOTINOODOMETRYNODE_H_

#include "ComROS.h"
#include "OdometryROS.h"

#include <ros/ros.h>

class RobotinoOdometryNode
{
public:
	RobotinoOdometryNode();
	~RobotinoOdometryNode();

	bool spin();

private:
	ros::NodeHandle nh_;

	std::string hostname_;

	ComROS com_;
	OdometryROS odometry_;

	void initModules();
};

#endif /* ROBOTINOODOMETRYNODE_H_ */
