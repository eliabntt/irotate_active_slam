#ifndef ROBOTINOCAM_PLUGIN
#define ROBOTINOCAM_PLUGIN

#include <map>
#include <string>
#include <stdlib.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>


#include <boost/thread/mutex.hpp>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <gazebo_msgs/ContactState.h>
#include <tf/tf.h>

#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/JointController.hh>

namespace gazebo {
    class Joint;

    class Entity;

    class robotinoCamPlugin : public ModelPlugin {
    public:
        robotinoCamPlugin();

        ~robotinoCamPlugin();

        void Load(physics::ModelPtr parent, sdf::ElementPtr);

    protected:
        void OnUpdate(const common::UpdateInfo &);

        void odomPublisher(double vel_z, double step_time);

        void FinishChild();
        void publishJointStates();

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void ekfCallback(const nav_msgs::Odometry::ConstPtr &msg);

        double GaussianKernel(double mu, double sigma);

        double CalculatePID(double goal, double current);

    private:
        void QueueThread();

        const double mean = 0.0;
        unsigned int seed;
        double stddev;
        std::default_random_engine generator;

        double vel_z, ekf_vel;
        double p,i,d,dt,integral,pre_error,max_v,min_v; // pid stuff

        double prev_w0, total_w;

        double odomCalc_pose;
        double prevUpdateTime, odom_rate;

        bool alive;

        boost::mutex lock;

        std::vector<physics::JointPtr> joints_;

        std::string prefix, cmd_vel_topic, robot_namespace, odom_topic, odom_frame, base_frame, ekf_topic;
        std::vector<std::string> joint_names_;

        physics::ModelPtr model;
        physics::WorldPtr world;

        event::ConnectionPtr updateConnection;

        ros::CallbackQueue queue;
        boost::shared_ptr<ros::NodeHandle> ros_node;
        boost::thread callback_queue;
        ros::Publisher pubCalc, pubMotor, logCamera;
        ros::Subscriber cmd_vel_subscriber;
        ros::Subscriber ekf_subscriber;

        nav_msgs::Odometry odomGT, odom;
    };
}

#endif