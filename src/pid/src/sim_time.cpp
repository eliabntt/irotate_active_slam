/***************************************************************************/ /**
 * \file sim_time.cpp
 *
 * \brief Node that publishes simulated time to the /clock topic
 *
 * \author Paul Bouchier
 * \date January 27, 2016
 *
 * \section license License (BSD-3)
 * Copyright (c) 2016, Paul Bouchier
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * - Neither the name of Willow Garage, Inc. nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

#include <sys/time.h>

#define SIM_TIME_INCREMENT_US 10000

/*
 * This node publishes increments of 1ms in time to the /clock topic. It does so
 * at a rate determined by sim_speedup (simulation speedup factor), which should
 * be passed
 * in as a private parameter.
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_time_source");
  ros::NodeHandle sim_time_node;

  // support integral multiples of wallclock time for simulation speedup
  int sim_speedup;  // integral factor by which to speed up simulation
  ros::NodeHandle node_priv("~");
  node_priv.param<int>("sim_speedup", sim_speedup, 1);

  // get the current time & populate sim_time with it
  struct timeval start;
  int rv = gettimeofday(&start, NULL);
  usleep(1000);
  struct timeval now;
  rv = gettimeofday(&now, NULL);
  if (0 != rv)
  {
    ROS_ERROR("Invalid return from gettimeofday: %d", rv);
    return -1;
  }

  rosgraph_msgs::Clock sim_time;
  sim_time.clock.sec = now.tv_sec - start.tv_sec;
  sim_time.clock.nsec = now.tv_usec * 1000;
  ros::Publisher sim_time_pub = sim_time_node.advertise<rosgraph_msgs::Clock>("clock", 1);

  ROS_INFO("Starting simulation time publisher at time: %d.%d", sim_time.clock.sec, sim_time.clock.nsec);

  while (ros::ok())
  {
    sim_time_pub.publish(sim_time);

    sim_time.clock.nsec = sim_time.clock.nsec + SIM_TIME_INCREMENT_US * 1000;
    while (sim_time.clock.nsec > 1000000000)
    {
      sim_time.clock.nsec -= 1000000000;
      ++sim_time.clock.sec;
    }

    usleep(SIM_TIME_INCREMENT_US / sim_speedup);
    ros::spinOnce();
  }
}