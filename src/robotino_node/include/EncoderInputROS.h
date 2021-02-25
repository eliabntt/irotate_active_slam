/*
 * EncoderInputROS.h
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ENCODERINPUTROS_H_
#define ENCODERINPUTROS_H_

#include "rec/robotino/api2/EncoderInput.h"

#include <ros/ros.h>
#include "robotino_msgs/EncoderReadings.h"
#include "robotino_msgs/SetEncoderPosition.h"


class EncoderInputROS: public rec::robotino::api2::EncoderInput
{
public:
	EncoderInputROS();
	~EncoderInputROS();

	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	ros::Publisher encoder_pub_;

	ros::ServiceServer encoder_position_server_;

	robotino_msgs::EncoderReadings encoder_msg_;

	ros::Time stamp_;

	void readingsChangedEvent( int velocity, int position, float current );

	bool setEncoderPositionCallback(
			robotino_msgs::SetEncoderPosition::Request& req,
			robotino_msgs::SetEncoderPosition::Response& res);
};

#endif /* ENCODERINPUTROS_H_ */
