/*
 * RobotinoNode.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "RobotinoNode.h"

RobotinoNode::RobotinoNode()
	: nh_("~")
{
	nh_.param<std::string>("hostname", hostname_, "172.26.1.1" );
	nh_.param<double>("max_linear_vel", max_linear_vel_, 0.2 );
	nh_.param<double>("min_linear_vel", min_linear_vel_, 0.05 );
	nh_.param<double>("max_angular_vel", max_angular_vel_, 1.0 );
	nh_.param<double>("min_angular_vel", min_angular_vel_, 0.1 );

	distances_clearing_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/distance_sensors_clearing", 1, true);
	joint_states_pub_= nh_.advertise<sensor_msgs::JointState>("/robotino_joint_states", 1, false);

	com_.setName( "RobotinoNode" );

	initModules();
	initMsgs();
}

RobotinoNode::~RobotinoNode()
{
	distances_clearing_pub_.shutdown();
	joint_states_pub_.shutdown();
}

void RobotinoNode::initModules()
{
	com_.setAddress( hostname_.c_str() );

	// Set the ComIds
	analog_input_array_.setComId( com_.id() );
	bumper_.setComId( com_.id() );
	digital_input_array_.setComId( com_.id() );
	digital_output_array_.setComId( com_.id() );
	distance_sensor_array_.setComId( com_.id() );
	encoder_input_.setComId( com_.id() );
	motor_array_.setComId( com_.id() );
	omni_drive_.setComId( com_.id() );
	power_management_.setComId( com_.id() );
	omni_drive_.setMaxMin(max_linear_vel_, min_linear_vel_, max_angular_vel_, min_angular_vel_ );
	com_.connectToServer( false );
}

void RobotinoNode::initMsgs()
{
	distances_clearing_msg_.header.frame_id = "base_link";
	distances_clearing_msg_.header.stamp = curr_time_;
	distances_clearing_msg_.points.resize( 720 );

	for( unsigned int i = 0; i < distances_clearing_msg_.points.size(); ++i )
	{
		distances_clearing_msg_.points[i].x = 5.0 * cos(  0.008727 ); // 0.008727 = 0.5 degrees in radians
		distances_clearing_msg_.points[i].y = 5.0 * sin(  0.008727 );
		distances_clearing_msg_.points[i].z = 0.05; // 5cm above ground
	}

	joint_state_msg_.name.resize(3);
	joint_state_msg_.position.resize(3, 0.0);
	joint_state_msg_.velocity.resize(3, 0.0);
	joint_state_msg_.name[0] = "wheel2_joint";
	joint_state_msg_.name[1] = "wheel0_joint";
	joint_state_msg_.name[2] = "wheel1_joint";

	motor_velocities_.resize(4);
	motor_positions_.resize(4);
}

void RobotinoNode::publishDistanceMsg()
{
//	curr_time_ = ros::Time::now();
//	if( ( curr_time_ - clearing_time_ ).toSec() > 1 )
//	{
//		clearing_time_ = curr_time_;
//		distances_clearing_pub_.publish( distances_clearing_msg_ );
//	}
	distances_clearing_pub_.publish( distances_clearing_msg_ );
}

void RobotinoNode::publishJointStateMsg()
{
	motor_array_.getMotorReadings( motor_velocities_, motor_positions_ );

	joint_state_msg_.velocity[0] = ( ( motor_velocities_[2] / 16 ) * (2 * 3.142) / 60 );
	joint_state_msg_.velocity[1] = ( ( motor_velocities_[0] / 16 ) * (2 * 3.142) / 60 );
	joint_state_msg_.velocity[2] = ( ( motor_velocities_[1] / 16 ) * (2 * 3.142) / 60 );

	joint_state_msg_.position[0] = ( motor_positions_[2] / 16 ) * (2 * 3.142);
	joint_state_msg_.position[1] = ( motor_positions_[0] / 16 ) * (2 * 3.142);
	joint_state_msg_.position[2] = ( motor_positions_[1] / 16 ) * (2 * 3.142);

	joint_state_msg_.header.stamp = curr_time_;
	joint_states_pub_.publish( joint_state_msg_ );
}

bool RobotinoNode::spin()
{
	ros::Rate loop_rate( 30 );

	while(nh_.ok())
	{
		curr_time_ = ros::Time::now();

		analog_input_array_.setTimeStamp(curr_time_);
		digital_input_array_.setTimeStamp(curr_time_);
		distance_sensor_array_.setTimeStamp(curr_time_);
		encoder_input_.setTimeStamp(curr_time_);
		motor_array_.setTimeStamp(curr_time_);
		power_management_.setTimeStamp(curr_time_);

		publishDistanceMsg();
		publishJointStateMsg();
		com_.processEvents();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return true;
}

