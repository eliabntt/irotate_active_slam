/*
 * KeyboardTeleop.cpp
 *
 *  Created on: 11.01.2012
 *      Author: indorewala@servicerobotics.eu
 */

#include "KeyboardTeleop.h"

KeyboardTeleop::KeyboardTeleop( struct termios &cooked, struct termios &raw, int &kfd ):
	nh_("~"),
	cooked_( cooked ),
	raw_( raw ),
	kfd_ ( kfd )
{
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
	readParams(nh_);
}

KeyboardTeleop::~KeyboardTeleop()
{
	cmd_vel_pub_.shutdown();
}

void KeyboardTeleop::readParams( ros::NodeHandle n )
{
	n.param<double>("scale_linear", scale_linear_, 1.0);
	n.param<double>("scale_angular", scale_angular_, 1.0);
}

void KeyboardTeleop::publish( double vel_x, double vel_y, double vel_omega )
{
	cmd_vel_msg_.linear.x 	= scale_linear_ * vel_x;
	cmd_vel_msg_.linear.y  	= scale_linear_ * vel_y;
	cmd_vel_msg_.angular.z 	= scale_angular_ * vel_omega;

	cmd_vel_pub_.publish( cmd_vel_msg_ );
}

void KeyboardTeleop::spin()
{
	char c;
	double vel_x, vel_y, vel_omega;

	// Get the console in raw mode
	tcgetattr( kfd_, &cooked_ );
	memcpy( &raw_, &cooked_, sizeof(struct termios) );
	raw_.c_lflag &=~ ( ICANON | ECHO );

	// Setting a new line, then end of file
	raw_.c_cc[VEOL] = 1;
	raw_.c_cc[VEOF] = 2;
	tcsetattr( kfd_, TCSANOW, &raw_ );

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use 'WASD' for translation");
	puts("Use 'QE' for rotation");

	puts("Press 'Space' to STOP");

	while( ros::ok() )
	{
		vel_x = 0.0;
		vel_y = 0.0;
		vel_omega = 0.0;

		// get the next event from the keyboard
		if(read( kfd_, &c, 1 ) < 0)
		{
			perror("read():");
			exit(-1);
		}

		switch( c )
		{
		// Walking
		case KEYCODE_W:
			vel_x = 0.05;
			break;
		case KEYCODE_S:
			vel_x = -0.05;
			break;
		case KEYCODE_A:
			vel_y = 0.05;
			break;
		case KEYCODE_D:
			vel_y = -0.05;
			break;
		case KEYCODE_Q:
			vel_omega = 0.5;
			break;
		case KEYCODE_E:
			vel_omega = -0.5;
			break;

		case KEYCODE_SPACE:
			vel_x = vel_y = vel_omega = 0.0;
			break;

		default:
			break;
		}

		boost::mutex::scoped_lock lock(publish_mutex_);
		if (ros::Time::now() > last_publish_ + ros::Duration(1.0))
		{
			first_publish_ = ros::Time::now();
		}
		last_publish_ = ros::Time::now();
		publish( vel_x, vel_y, vel_omega );
	}
	return;
}

void KeyboardTeleop::watchdog()
{
	boost::mutex::scoped_lock lock( publish_mutex_ );
	if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) &&
			(ros::Time::now() > first_publish_ + ros::Duration(0.50)))
		publish(0.0, 0.0, 0.0);
}
