/*
 * DigitalInputArrayROS.cpp
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "DigitalInputArrayROS.h"

DigitalInputArrayROS::DigitalInputArrayROS()
{
	digital_pub_ = nh_.advertise<robotino_msgs::DigitalReadings>("digital_readings", 1, true);
}

DigitalInputArrayROS::~DigitalInputArrayROS()
{
	digital_pub_.shutdown();
}

void DigitalInputArrayROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void DigitalInputArrayROS::valuesChangedEvent( const int* values, unsigned int size )
{
	// Build the DigitalReadings msg
	digital_msg_.stamp = stamp_;
	digital_msg_.values.resize( size );

	if( size > 0 )
	{
        for( unsigned int idx = 0; idx < size; ++idx )
        {
            digital_msg_.values[idx] = (bool)values[idx];
        }
		// Publish the msg
		digital_pub_.publish( digital_msg_ );
	}

}
