/*
 * main.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */


#include "RobotinoLaserRangeFinderNode.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_laserrangefinder_node");
	RobotinoLaserRangeFinderNode rn;
	rn.spin();
	return 0;
}
