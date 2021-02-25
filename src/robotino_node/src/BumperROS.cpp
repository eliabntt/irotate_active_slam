/*
 * BumperROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */


#include "BumperROS.h"

BumperROS::BumperROS()
{
	bumper_pub_ = nh_.advertise<std_msgs::Bool>("bumper", 1, true);
}

BumperROS::~BumperROS()
{
	bumper_pub_.shutdown();
}

void BumperROS::bumperEvent(bool hasContact)
{
	bumper_msg_.data = hasContact;
	bumper_pub_.publish(bumper_msg_);
}
