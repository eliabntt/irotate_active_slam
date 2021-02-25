#include "InitialPoseROS.h"

InitialPoseROS::InitialPoseROS()
{
	initialPose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
	mapInfo_ = NULL;
	map_sub_ = nh_.subscribe("map", 1, &InitialPoseROS::mapCallback, this);

}

InitialPoseROS::~InitialPoseROS()
{
	initialPose_pub_.shutdown();
	map_sub_.shutdown();
}

void InitialPoseROS:: initialPoseEvent(float x,float y,double r)
{
	if(mapInfo_)
	{
		initialPose_msg_.pose.pose.position.x = ( -mapInfo_->resolution * (x + mapInfo_->offset[0] ) );
		initialPose_msg_.pose.pose.position.y = ( mapInfo_->resolution * ( y + mapInfo_->offset[1] ) );
		initialPose_msg_.pose.pose.position.z = 0;
	
		double rot = deg2rad( r );
		double rx = cos( rot );
		double ry = sin( rot );
		rot = atan2( ry, -rx );

		tf::Quaternion q = tf::createQuaternionFromYaw( rot );
		initialPose_msg_.pose.pose.orientation.x = q.x();
		initialPose_msg_.pose.pose.orientation.y = q.y();
		initialPose_msg_.pose.pose.orientation.z = q.z();
		initialPose_msg_.pose.pose.orientation.w = q.w();	

		initialPose_msg_.header.frame_id = mapInfo_->frame_id;
		initialPose_msg_.header.stamp = ros::Time::now();

		initialPose_pub_.publish(initialPose_msg_);
	}
}

void InitialPoseROS::mapCallback(const nav_msgs::OccupancyGrid& occupancyGrid)
{
	if(mapInfo_)
	{
		delete mapInfo_;
	}
	mapInfo_ = new MapInfo(occupancyGrid, occupancyGrid.header.frame_id);
}
