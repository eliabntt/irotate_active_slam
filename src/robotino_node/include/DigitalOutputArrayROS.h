/*
 * DigitalOutputArrayROS.h
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef DIGITALOUTPUTARRAYROS_H_
#define DIGITALOUTPUTARRAYROS_H_

#include "rec/robotino/api2/DigitalOutputArray.h"

#include <ros/ros.h>
#include "robotino_msgs/DigitalReadings.h"

class DigitalOutputArrayROS: public rec::robotino::api2::DigitalOutputArray
{
public:
	DigitalOutputArrayROS();
	~DigitalOutputArrayROS();

private:
	ros::NodeHandle nh_;

	ros::Subscriber digital_sub_;

	void setDigitalValuesCallback( const robotino_msgs::DigitalReadingsConstPtr& msg);

};

#endif /* DIGITALOUTPUTARRAYROS_H_ */
