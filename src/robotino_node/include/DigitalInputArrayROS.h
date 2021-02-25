/*
 * DigitalInputArrayROS.h
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef DIGITALINPUTARRAYROS_H_
#define DIGITALINPUTARRAYROS_H_

#include "rec/robotino/api2/DigitalInputArray.h"

#include <ros/ros.h>
#include "robotino_msgs/DigitalReadings.h"

class DigitalInputArrayROS: public rec::robotino::api2::DigitalInputArray
{
public:
	DigitalInputArrayROS();
	~DigitalInputArrayROS();

	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	ros::Publisher digital_pub_;

	robotino_msgs::DigitalReadings digital_msg_;

	ros::Time stamp_;

	void valuesChangedEvent( const int* values, unsigned int size );

};

#endif /* DIGITALINPUTARRAYROS_H_ */
