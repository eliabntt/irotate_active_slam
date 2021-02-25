#ifndef ROBOTINO_PLUGIN
#define ROBOTINO_PLUGIN

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

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>


#include <boost/thread/mutex.hpp>

#include <std_msgs/String.h>

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
#include <sensor_msgs/JointState.h>

namespace gazebo {
    class Joint;

    class Entity;

    class robotinoPlugin : public ModelPlugin {
    public:
        robotinoPlugin();

        ~robotinoPlugin();

        void Load(physics::ModelPtr parent, sdf::ElementPtr);

    protected:
        void OnUpdate(const common::UpdateInfo &);

        void odomPublisher(double vel_z, double step_time);

        void FinishChild();

        void publishJointStates();

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

        void getVelInGlobFrame(double w0, double w1, double w2, double angle, double &vx_glob, double &vy_glob,
                               double &vz_glob);

        double GaussianKernel(double mu, double sigma);

    private:
        void QueueThread();

        const double mean = 0.0;
        unsigned int seed;
        double stddev;
        std::default_random_engine generator;

        double vel_x, vel_y, vel_z;

        double prev_w0, prev_w1, prev_w2;

        double odomCalc_pose[3];
        double prevUpdateTime, odom_rate;
        double vx_noisy, vy_noisy, vw_noisy;

        double total_w0, total_w1, total_w2;

        bool alive;

        boost::mutex lock;

        ignition::math::Pose3d pose3D, prev_odom_pose;

        std::vector<physics::JointPtr> joints_;

        std::string prefix, cmd_vel_topic, robot_namespace, odom_topic, odom_frame, base_frame;
        std::vector<std::string> joint_names_;

        physics::ModelPtr model;
        physics::WorldPtr world;

        event::ConnectionPtr updateConnection;

        ros::CallbackQueue queue;
        boost::shared_ptr<ros::NodeHandle> ros_node;
        boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
        boost::thread callback_queue;
        ros::Publisher pubGT, pubCalc, joint_state_publisher_, wheels;
        ros::Subscriber cmd_vel_subscriber;

        sensor_msgs::JointState joint_state_;
        nav_msgs::Odometry odomGT, odom;

    };
}

#endif