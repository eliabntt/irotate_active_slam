/*
 * MotorArrayROS.h
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef MOTORARRAYROS_H_
#define MOTORARRAYROS_H_

#include "rec/robotino/api2/MotorArray.h"

#include <ros/ros.h>
#include "robotino_msgs/MotorReadings.h"

class MotorArrayROS : public rec::robotino::api2::MotorArray
{
public:
	MotorArrayROS();
	~MotorArrayROS();

	void setTimeStamp(ros::Time stamp);
	void getMotorReadings(std::vector<float> &velocities, std::vector<int> &positions );

private:
	ros::NodeHandle nh_;

	ros::Publisher motor_pub_;

	robotino_msgs::MotorReadings motor_msg_;

	ros::Time stamp_;

	void velocitiesChangedEvent( const float* velocities, unsigned int size );
	void positionsChangedEvent( const float* positions, unsigned int size );
	void currentsChangedEvent( const float* currents, unsigned int size );
};
#endif /* MOTORARRAYROS_H_ */
