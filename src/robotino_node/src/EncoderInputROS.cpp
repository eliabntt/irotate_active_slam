/*
 * EncoderInputROS.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "EncoderInputROS.h"

EncoderInputROS::EncoderInputROS()
{
	encoder_pub_ = nh_.advertise<robotino_msgs::EncoderReadings>("encoder_readings", 1, true);
	encoder_position_server_ = nh_.advertiseService("set_encoder_position",
			&EncoderInputROS::setEncoderPositionCallback, this);
}

EncoderInputROS::~EncoderInputROS()
{
	encoder_pub_.shutdown();
	encoder_position_server_.shutdown();
}

void EncoderInputROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void EncoderInputROS::readingsChangedEvent( int velocity, int position, float current )
{
	// Build the EncoderReadings msg
	encoder_msg_.stamp = stamp_;
	encoder_msg_.velocity = velocity;
	encoder_msg_.position = position;
	encoder_msg_.current = current;

	// Publish the msg
	encoder_pub_.publish( encoder_msg_ );
}

bool EncoderInputROS::setEncoderPositionCallback(
			robotino_msgs::SetEncoderPosition::Request& req,
			robotino_msgs::SetEncoderPosition::Response& res)
{
	setPosition( req.position ,req.velocity );

	return true;
}
