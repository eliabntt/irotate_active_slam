/*
 * CameraROS.h
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef CAMERAROS_H_
#define CAMERAROS_H_

#include "rec/robotino/api2/Camera.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

class CameraROS : public rec::robotino::api2::Camera
{
public:
	CameraROS();
	~CameraROS();

	void setNumber( int number );
	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	image_transport::ImageTransport img_transport_;
	image_transport::CameraPublisher streaming_pub_;

	sensor_msgs::Image img_msg_;
	sensor_msgs::CameraInfo cam_info_msg_;

	ros::Time stamp_;

	void imageReceivedEvent(
			const unsigned char* data,
			unsigned int dataSize,
			unsigned int width,
			unsigned int height,
			unsigned int step );

};

#endif /* CAMERAROS_H_ */
