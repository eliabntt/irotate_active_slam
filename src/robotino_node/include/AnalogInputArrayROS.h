/*
 * AnalogInputArrayROS.h
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ANALOGINPUTARRAYROS_H_
#define ANALOGINPUTARRAYROS_H_

#include "rec/robotino/api2/AnalogInputArray.h"

#include <ros/ros.h>
#include "robotino_msgs/AnalogReadings.h"

class AnalogInputArrayROS: public rec::robotino::api2::AnalogInputArray
{
public:
	AnalogInputArrayROS();
	~AnalogInputArrayROS();

	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	ros::Publisher analog_pub_;

	robotino_msgs::AnalogReadings analog_msg_;

	ros::Time stamp_;

	void valuesChangedEvent( const float* values, unsigned int size );

};


#endif /* ANALOGINPUTARRAYROS_H_ */
