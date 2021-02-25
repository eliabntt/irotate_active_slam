/*
 * RobotinoNode.h
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef RobotinoCameraNode_H
#define RobotinoCameraNode_H

#include "ComROS.h"
#include "CameraROS.h"

#include <ros/ros.h>

class RobotinoCameraNode
{
public:
	RobotinoCameraNode();
	~RobotinoCameraNode();

	bool spin();

private:
	ros::NodeHandle nh_;

	std::string hostname_;
	int cameraNumber_;

	ComROS com_;
	CameraROS camera_;

	void initModules();
};

#endif /* RobotinoCameraNode_H */
