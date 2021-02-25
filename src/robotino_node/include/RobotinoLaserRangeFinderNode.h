/*
 * RobotinoNode.h
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ROBOTINOLaserRangeFinderNODE_H_
#define ROBOTINOLaserRangeFinderNODE_H_

#include "ComROS.h"
#include "LaserRangeFinderROS.h"

#include <ros/ros.h>

class RobotinoLaserRangeFinderNode
{
public:
	RobotinoLaserRangeFinderNode();
	~RobotinoLaserRangeFinderNode();

	bool spin();

private:
	ros::NodeHandle nh_;

	std::string hostname_;
	int laserRangeFinderNumber_;

	ComROS com_;
	LaserRangeFinderROS laser_range_finder_;

	void initModules();
};

#endif /* ROBOTINOLaserRangeFinderNODE_H_ */
