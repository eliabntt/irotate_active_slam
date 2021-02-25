#ifndef INITIALPOSEROS_H_
#define INITIALPOSEROS_H_

#include "transform.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

class InitialPoseROS: public rec::robotino::api2::InitialPose
{
public:
	InitialPoseROS();
	~InitialPoseROS();

private:
	ros::NodeHandle nh_;

	ros::Subscriber map_sub_;

	ros::Publisher initialPose_pub_;

	geometry_msgs::PoseWithCovarianceStamped initialPose_msg_;

	void initialPoseEvent(float x,float y,double r);

	void mapCallback(const nav_msgs::OccupancyGrid& occupancyGrid);

	MapInfo* mapInfo_;
};

#endif /* INITIALPOSEROS_H_ */
