/*
 * CameraROS.cpp
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "CameraROS.h"
#include <sensor_msgs/fill_image.h>

namespace sensor_msgs
{
extern bool fillImage(Image &image, const std::string &encoding_arg, uint32_t rows_arg, uint32_t cols_arg, uint32_t step_arg, const void *data_arg);
};


CameraROS::CameraROS():
	img_transport_(nh_)
{
}

CameraROS::~CameraROS()
{
	streaming_pub_.shutdown();
}

void CameraROS::setNumber( int number )
{
	std::stringstream topic;

	if( number == 0)
		topic << "image_raw";
	else
		topic << "image_raw" << number;

	streaming_pub_ = img_transport_.advertiseCamera(topic.str(), 1, false);

	setCameraNumber( number );
}

void CameraROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void CameraROS::imageReceivedEvent(
		const unsigned char* data,
		unsigned int dataSize,
		unsigned int width,
		unsigned int height,
		unsigned int step )
{
	// Build the Image msg
	img_msg_.header.stamp = stamp_;
	sensor_msgs::fillImage(img_msg_, "bgr8", height, width, step, data);

	// Build the CameraInfo msg
	cam_info_msg_.header.stamp = stamp_;
	cam_info_msg_.height = height;
	cam_info_msg_.width = width;

	// Publish the Image & CameraInfo msgs
	streaming_pub_.publish(img_msg_, cam_info_msg_);

}
