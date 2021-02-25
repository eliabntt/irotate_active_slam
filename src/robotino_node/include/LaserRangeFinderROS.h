/*
 * LaserRangeFinderROS.h
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef LASERRANGEFINDERROS_H_
#define LASERRANGEFINDERROS_H_

#include "rec/robotino/api2/LaserRangeFinder.h"
#include "rec/robotino/api2/LaserRangeFinderReadings.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserRangeFinderROS: public rec::robotino::api2::LaserRangeFinder
{
public:
	LaserRangeFinderROS();
	~LaserRangeFinderROS();

	void setNumber( int number );
	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	ros::Publisher laser_scan_pub_;

	sensor_msgs::LaserScan laser_scan_msg_;

	ros::Time stamp_;

	void scanEvent(const rec::robotino::api2::LaserRangeFinderReadings &scan);
};

#endif /* LASERRANGEFINDERROS_H_ */
