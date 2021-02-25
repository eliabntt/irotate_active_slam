/*
 * keyboard_teleop_node.cpp
 *
 *  Created on: 11.01.2012
 *      Author: indorewala@servicerobotics.eu
 */

#include "KeyboardTeleop.h"

#include <termios.h>
#include <signal.h>

#include "boost/thread/thread.hpp"

int kfd = 0;
struct termios cooked, raw;

#include <ros/ros.h>

void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	exit(0);
}

int main( int argc, char** argv )
{
	ros::init( argc, argv, "keyboard_teleop_node" );
	ros::NodeHandle n;

	signal(SIGINT,quit);

	KeyboardTeleop kt( cooked, raw, kfd );

	boost::thread my_thread(boost::bind(&KeyboardTeleop::spin, &kt));
	ros::Timer timer =
			n.createTimer(ros::Duration(0.1), boost::bind(&KeyboardTeleop::watchdog, &kt));

	ros::spin();

	my_thread.interrupt();
	my_thread.join();

	return 0;
}
