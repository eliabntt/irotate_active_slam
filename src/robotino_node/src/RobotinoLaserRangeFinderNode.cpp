/*
 * RobotinoNode.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "RobotinoLaserRangeFinderNode.h"
#include <sstream>

RobotinoLaserRangeFinderNode::RobotinoLaserRangeFinderNode()
	: nh_("~")
{
	nh_.param<std::string>("hostname", hostname_, "172.26.1.1" );
	nh_.param<int>("laserRangeFinderNumber", laserRangeFinderNumber_, 0 );

	std::ostringstream os;
	os << "LaserRangeFinder" << laserRangeFinderNumber_;
	com_.setName( os.str() );

	initModules();
}

RobotinoLaserRangeFinderNode::~RobotinoLaserRangeFinderNode()
{
}

void RobotinoLaserRangeFinderNode::initModules()
{
	com_.setAddress( hostname_.c_str() );

	// Set the ComIds
	laser_range_finder_.setComId( com_.id() );

	// Set the LaserRangeFinder numbers
	laser_range_finder_.setNumber( laserRangeFinderNumber_ );

	com_.connectToServer( false );
}

bool RobotinoLaserRangeFinderNode::spin()
{
	ros::Rate loop_rate( 30 );

	while(nh_.ok())
	{
		ros::Time curr_time = ros::Time::now();
		laser_range_finder_.setTimeStamp(curr_time);

		com_.processEvents();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return true;
}

