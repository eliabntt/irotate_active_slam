#ifndef _TRANSFORM_H_
#define _TRANSFORM_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

static const float PI = 3.14159265358979323846f;

#ifndef FUNCTION_IS_NOT_USED
#ifdef __GNUC__
#define FUNCTION_IS_NOT_USED __attribute__ ((unused))
#else
#define FUNCTION_IS_NOT_USED
#endif
#endif

class MapInfo
{
 public:
 MapInfo(): resolution( 0.0f )
		, frame_id( "/map" )
	{
		offset[0] = 0.0f;
		offset[1] = 0.0f;
		width = 800;
		height = 800;
	}
	

	MapInfo( const nav_msgs::OccupancyGridConstPtr& msg, std::string frame_id_ = "/map" )
	{
		resolution = msg->info.resolution;
		offset[0] = -((int)msg->info.width) - (msg->info.origin.position.x / resolution);	// remember: horizontal mirroring!
		offset[1] = msg->info.origin.position.y / resolution;
		frame_id = frame_id_;
		width = (int)msg->info.width;
		height = (int)msg->info.height;
	}

	MapInfo( const nav_msgs::OccupancyGrid& msg, std::string frame_id_ = "/map" )
	{
		resolution = msg.info.resolution;
		offset[0] = -((int)msg.info.width) - (msg.info.origin.position.x / resolution);	// remember: horizontal mirroring!
		offset[1] = msg.info.origin.position.y / resolution;
		frame_id = frame_id_;

		width = (int)msg.info.width;
		height = (int)msg.info.height;
	}

	bool isEmpty() const { return 0.0f == resolution; }

	void clear() { resolution = 0.0f; }

	float resolution;
	float offset[2];
	int width;
	int height;
	std::string frame_id;
};

static FUNCTION_IS_NOT_USED float deg2rad( float deg )
{
	return PI * deg / 180.0f;
}

static FUNCTION_IS_NOT_USED float rad2deg( float rad )
{
	return 180.0f * rad / PI;
}

static FUNCTION_IS_NOT_USED double getYaw( const tf::Pose& p )
{
	tfScalar useless_pitch, useless_roll, yaw;
	p.getBasis().getRPY( useless_roll, useless_pitch, yaw );

	double x = cos( yaw );
	double y = sin( yaw );

	return rad2deg( atan2( y, -x ) );
}

static FUNCTION_IS_NOT_USED bool poseToMap( 
					   tf::TransformListener* tf,  const MapInfo& mapInfo,
					   const nav_msgs::Odometry& odom, float* x,float*y, double* rotation_deg )
{
	try
	{
		tf::StampedTransform transform;
		tf->lookupTransform( mapInfo.frame_id, odom.header.frame_id, ros::Time(0), transform );
		
		tf::Pose p;
		p.setOrigin( tf::Vector3( odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z ) );
		p.setRotation( tf::Quaternion( odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w ) );

		p = transform * p;

		if( rotation_deg )
		{
			*rotation_deg = getYaw( p );
		}
		*x = (-1.0 / mapInfo.resolution * p.getOrigin()[0] - mapInfo.offset[0]) ;
		*y = (1.0 / mapInfo.resolution * p.getOrigin()[1] - mapInfo.offset[1]);
		return true;
	}
	catch(tf::LookupException& ex) {
		ROS_ERROR("No Transform available Error: %s\n", ex.what());
		return false;
	}
	catch(tf::ConnectivityException& ex) {
		ROS_ERROR("Connectivity Error: %s\n", ex.what());
		return false;
	}
	catch(tf::ExtrapolationException& ex) {
		ROS_ERROR("Extrapolation Error: %s\n", ex.what());
		return false;
	}

	return false;
}

#endif
