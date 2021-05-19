/***************************************************************************/ /**
 * \file plant_sim.h
 *
 * \brief First or second order plant simulator
 * \author Andy Zelenak
 * \author Paul Bouchier
 * \date March 8, 2015
 *
 * \section license License (BSD-3)
 * Copyright (c) 2015, Paul Bouchier\n
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
 *
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

// This file simulates a 1st or 2nd-order dynamic system, publishes its state,
// and subscribes to a 'control_effort' topic. The control effort is used
// to stabilize the servo.

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace plant_sim
{
// Global so it can be passed from the callback fxn to main
static double control_effort = 0.0;
static bool reverse_acting = false;
}
using namespace plant_sim;

// Callback when something is published on 'control_effort'
void controlEffortCallback(const std_msgs::Float64& control_effort_input)
{
  // the stabilizing control effort
  if (reverse_acting)
  {
    control_effort = -control_effort_input.data;
  }
  else
  {
    control_effort = control_effort_input.data;
  }
}

int main(int argc, char** argv)
{
  int plant_order = 1;
  double temp = 4.7;          // initial condition for first-order plant
  double displacement = 3.3;  // initial condition for second-order plant

  ros::init(argc, argv, "plant");
  ros::NodeHandle sim_node;

  while (ros::ok() && ros::Time(0) == ros::Time::now())
  {
    ROS_INFO("Plant_sim spinning waiting for time to become non-zero");
    sleep(1);
  }

  ros::NodeHandle node_priv("~");
  node_priv.param<int>("plant_order", plant_order, 1);
  node_priv.param<bool>("reverse_acting", reverse_acting, false);

  if (plant_order == 1)
  {
    ROS_INFO("Starting simulation of a first-order plant.");
  }
  else if (plant_order == 2)
  {
    ROS_INFO("Starting simulation of a second-order plant.");
  }
  else
  {
    ROS_ERROR("Error: Invalid plant type parameter, must be 1 or 2: %s", argv[1]);
    return -1;
  }

  // Advertise a plant state msg
  std_msgs::Float64 plant_state;
  ros::Publisher servo_state_pub = sim_node.advertise<std_msgs::Float64>("state", 1);

  // Subscribe to "control_effort" topic to get a controller_msg.msg
  ros::Subscriber sub = sim_node.subscribe("control_effort", 1, controlEffortCallback);

  int loop_counter = 0;
  double delta_t = 0.01;
  ros::Rate loop_rate(1 / delta_t);  // Control rate in Hz

  // Initialize 1st-order (e.g temp controller) process variables
  double temp_rate = 0;  // rate of temp change

  // Initialize 2nd-order (e.g. servo-motor with load) process variables
  double speed = 0;         // meters/sec
  double acceleration = 0;  // meters/sec^2
  double mass = 0.1;        // in kg
  double friction = 1.0;    // a decelerating force factor
  double stiction = 1;      // control_effort must exceed this before stationary servo moves
  double Kv = 1;            // motor constant: force (newtons) / volt
  double Kbackemf = 0;      // Volts of back-emf per meter/sec of speed
  double decel_force;       // decelerating force

  while (ros::ok())
  {
    ros::spinOnce();

    switch (plant_order)
    {
      case 1:  // First order plant
        temp_rate = (0.1 * temp) + control_effort;
        temp = temp + temp_rate * delta_t;

        plant_state.data = temp;
        break;

      case 2:  // Second order plant
        if (fabs(speed) < 0.001)
        {
          // if nearly stopped, stop it & require overcoming stiction to restart
          speed = 0;
          if (fabs(control_effort) < stiction)
          {
            control_effort = 0;
          }
        }

        // Update the servo.
        // control_effort: the voltage applied to the servo. Output from PID
        // controller. It is
        //   opposed by back emf (expressed as speed) to produce a net force.
        //   Range: -1 to +1
        // displacement: the actual value of the servo output position. Input to
        // PID controller

        decel_force = -(speed * friction);  // can be +ve or -ve. Linear with speed
        acceleration = ((Kv * (control_effort - (Kbackemf * speed)) + decel_force) / mass);  // a = F/m
        speed = speed + (acceleration * delta_t);
        displacement = displacement + speed * delta_t;

        plant_state.data = displacement;
        break;

      default:
        ROS_ERROR("Invalid plant_order: %d", plant_order);
        return (-1);
    }

    servo_state_pub.publish(plant_state);
    loop_rate.sleep();
  }

  return 0;
}
