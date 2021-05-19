/***************************************************************************/ /**
 * \file autotune.cpp
 *
 * \brief Autotune a PID controller with the Ziegler Nichols method
 * \author Andy Zelenak
 * \date October 25, 2016
 *
 * \section license License (BSD-3)
 * Copyright (c) 2016, Andy Zelenak\n
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

// Use the Ziegler Nichols Method to calculate reasonable PID gains.
// The ZN Method is based on setting Ki & Kd to zero, then cranking up Kp until
// oscillations are observed.
// This node varies Kp through a range until oscillations are just barely
// observed,
// then calculates the other parameters automatically.
// See https://en.wikipedia.org/wiki/PID_controller

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>

// Use dynamic_reconfigure to adjust Kp, Ki, and Kd
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

void setKiKdToZero();
void setKp(double Kp);
void setpointCallback(const std_msgs::Float64& setpoint_msg);
void stateCallback(const std_msgs::Float64& state_msg);
void setFinalParams();

namespace autotune
{
double Ku = 0.;
double Tu = 0.;
double setpoint = 0.;
double state = 0.;
std::string ns = "/left_wheel_pid/";
int oscillation_count = 0;
int num_loops = 100;  // Will look for oscillations for num_loops*loopRate
int initial_error = 0;
double Kp_ZN = 0.;
double Ki_ZN = 0.;
double Kd_ZN = 0.;
bool found_Ku = false;
std::vector<double> oscillation_times(3);  // Used to calculate Tu, the oscillation period
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "autotune_node");
  ros::NodeHandle autotune_node;
  ros::start();
  ros::Subscriber setpoint_sub = autotune_node.subscribe("/setpoint", 1, setpointCallback);
  ros::Subscriber state_sub = autotune_node.subscribe("/state", 1, stateCallback);
  ros::Rate loopRate(50);

  // Set Ki and Kd to zero for the ZN method with dynamic_reconfigure
  setKiKdToZero();

  // Define how rapidly the value of Kp is varied, and the max/min values to try
  double Kp_max = 10.;
  double Kp_min = 0.5;
  double Kp_step = 1.0;

  for (double Kp = Kp_min; Kp <= Kp_max; Kp += Kp_step)
  {
    //////////////////////
    // Get the error sign.
    //////////////////////
    // Need to wait for new setpoint/state msgs
    ros::topic::waitForMessage<std_msgs::Float64>("setpoint");
    ros::topic::waitForMessage<std_msgs::Float64>("state");

    // Try a new Kp.
    setKp(Kp);
    ROS_INFO_STREAM("Trying Kp = " << Kp);  // Blank line on terminal
    autotune::oscillation_count = 0;        // Reset to look for oscillations again

    for (int i = 0; i < autotune::num_loops; i++)  // Collect data for loopRate*num_loops seconds
    {
      ros::spinOnce();
      loopRate.sleep();
      if (i == 0)  // Get the sign of the initial error
      {
        autotune::initial_error = (autotune::setpoint - autotune::state);
      }

      // Did the system oscillate about the setpoint? If so, Kp~Ku.
      // Oscillation => the sign of the error changes
      // The first oscillation is caused by the change in setpoint. Ignore it.
      // Look for 2 oscillations.
      // Get a fresh state message
      ros::topic::waitForMessage<std_msgs::Float64>("state");
      double new_error = (autotune::setpoint - autotune::state);  // Sign of the error
      // ROS_INFO_STREAM("New error: "<< new_error);
      if (std::signbit(autotune::initial_error) != std::signbit(new_error))
      {
        autotune::oscillation_times.at(autotune::oscillation_count) =
            loopRate.expectedCycleTime().toSec() * i;  // Record the time to calculate a period, Tu
        autotune::oscillation_count++;
        ROS_INFO_STREAM("Oscillation occurred. Oscillation count:  " << autotune::oscillation_count);
        autotune::initial_error = new_error;  // Reset to look for another oscillation

        // If the system is definitely oscillating about the setpoint
        if (autotune::oscillation_count > 2)
        {
          // Now calculate the period of oscillation (Tu)
          autotune::Tu = autotune::oscillation_times.at(2) - autotune::oscillation_times.at(0);
          ROS_INFO_STREAM("Tu (oscillation period): " << autotune::Tu);
          // ROS_INFO_STREAM( "2*sampling period: " <<
          // 2.*loopRate.expectedCycleTime().toSec() );

          // We're looking for more than just the briefest dip across the
          // setpoint and back.
          // Want to see significant oscillation
          if (autotune::Tu > 3. * loopRate.expectedCycleTime().toSec())
          {
            autotune::Ku = Kp;

            // Now calculate the other parameters with ZN method
            autotune::Kp_ZN = 0.6 * autotune::Ku;
            autotune::Ki_ZN = 1.2 * autotune::Ku / autotune::Tu;
            autotune::Kd_ZN = 3. * autotune::Ku * autotune::Tu / 40.;

            autotune::found_Ku = true;
            goto DONE;
          }
          else
            break;  // Try the next Kp
        }
      }
    }
  }
DONE:

  if (autotune::found_Ku == true)
  {
    setFinalParams();
  }
  else
    ROS_INFO_STREAM("Did not see any oscillations for this range of Kp. Adjust "
                    "Kp_max and Kp_min to broaden the search.");

  ros::shutdown();
  return 0;
}

// Set Ki and Kd to zero with dynamic_reconfigure
void setKiKdToZero()
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config config;
  double_param.name = "Ki";
  double_param.value = 0.0;
  config.doubles.push_back(double_param);
  double_param.name = "Kd";
  double_param.value = 0.0;
  config.doubles.push_back(double_param);
  srv_req.config = config;
  ros::service::call(autotune::ns + "set_parameters", srv_req, srv_resp);
}

// Set Kp with dynamic_reconfigure
void setKp(double Kp)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config config;

  // A blank service call to get the current parameters into srv_resp
  ros::service::call(autotune::ns + "set_parameters", srv_req, srv_resp);

  double_param.name = "Kp";
  double_param.value = Kp / srv_resp.config.doubles.at(0).value;  // Adjust for the scale slider on the GUI
  config.doubles.push_back(double_param);
  srv_req.config = config;
  ros::service::call(autotune::ns + "set_parameters", srv_req, srv_resp);
}

void setpointCallback(const std_msgs::Float64& setpoint_msg)
{
  autotune::setpoint = setpoint_msg.data;
}

void stateCallback(const std_msgs::Float64& state_msg)
{
  autotune::state = state_msg.data;
}

// Print out and set the final parameters as calculated by the autotuner
void setFinalParams()
{
  ROS_INFO_STREAM(" ");
  ROS_INFO_STREAM("The suggested parameters are: ");
  ROS_INFO_STREAM("Kp  " << autotune::Kp_ZN);
  ROS_INFO_STREAM("Ki  " << autotune::Ki_ZN);
  ROS_INFO_STREAM("Kd  " << autotune::Kd_ZN);

  // Set the ZN parameters with dynamic_reconfigure
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config config;

  // A blank service call to get the current parameters into srv_resp
  ros::service::call(autotune::ns + "set_parameters", srv_req, srv_resp);

  double_param.name = "Kp";
  double_param.value = autotune::Kp_ZN / srv_resp.config.doubles.at(0).value;  // Adjust for the scale slider on the GUI
  config.doubles.push_back(double_param);

  double_param.name = "Ki";
  double_param.value = autotune::Ki_ZN / srv_resp.config.doubles.at(0).value;  // Adjust for the scale slider on the GUI
  config.doubles.push_back(double_param);

  double_param.name = "Kd";
  double_param.value = autotune::Kd_ZN / srv_resp.config.doubles.at(0).value;  // Adjust for the scale slider on the GUI
  config.doubles.push_back(double_param);

  srv_req.config = config;
  ros::service::call(autotune::ns + "set_parameters", srv_req, srv_resp);
}