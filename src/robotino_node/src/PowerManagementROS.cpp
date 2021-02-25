/*
 * PowerManagementROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "PowerManagementROS.h"

PowerManagementROS::PowerManagementROS()
{
	power_pub_ = nh_.advertise<robotino_msgs::PowerReadings>("power_readings", 1, true);
}

PowerManagementROS::~PowerManagementROS()
{
	power_pub_.shutdown();
}

void PowerManagementROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void PowerManagementROS::readingsEvent(float current, float voltage)
{
	// Build the PowerReadings msg
	power_msg_.stamp = ros::Time::now();
	power_msg_.current = current;
	power_msg_.voltage = voltage;

	// Publish the msg
	power_pub_.publish( power_msg_ );
}
