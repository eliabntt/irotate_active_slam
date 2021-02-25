#include <robotino_gazebo_plugin/robotino_plugin.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <control_toolbox/pid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define MATH_PI     3.1415926535897931
#define SIN_PI_3    sinf(MATH_PI/3)
#define COS_PI_3    cosf(MATH_PI/3)
#define R_WHEEL     0.04
#define R_BASE      0.17

namespace gazebo {

    robotinoPlugin::robotinoPlugin()
            : ModelPlugin(), vel_x(0), vel_y(0), vel_z(0) {
        ROS_INFO("Starting robotino_gazebo plugin");
        this->seed = 0;
    }

    robotinoPlugin::~robotinoPlugin() {
        FinishChild();
    }


    void robotinoPlugin::QueueThread() {
        static const double timeout = 0.01;
        while (alive && ros_node->ok()) {
            queue.callAvailable(ros::WallDuration(timeout));
        }
    }

    void robotinoPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
        // Store the pointer to the model
        this->model = parent;
        this->world = parent->GetWorld();

        total_w0 = 0;
        total_w1 = 0;
        total_w2 = 0;

        robot_namespace = "";
        if (!sdf->HasElement("robotNamespace")) {
            ROS_INFO_NAMED("robotino_move", "RobotinoGazeboPlugin missing <robotNamespace>, "
                                            "defaults to \"%s\"", robot_namespace.c_str());
        } else {
            robot_namespace =
                    sdf->GetElement("robotNamespace")->Get<std::string>();
        }

        cmd_vel_topic = "cmd_vel";
        if (!sdf->HasElement("commandTopic")) {
            ROS_WARN_NAMED("robotino_move", "RobotinoGazeboPlugin (ns = %s) missing <commandTopic>, "
                                            "defaults to \"%s\"",
                           robot_namespace.c_str(), cmd_vel_topic.c_str());
        } else {
            cmd_vel_topic = sdf->GetElement("commandTopic")->Get<std::string>();
        }

        odom_topic = "odom";
        if (!sdf->HasElement("odometryTopic")) {
            ROS_WARN_NAMED("robotino_move", "RobotinoGazeboPlugin (ns = %s) missing <odometryTopic>, "
                                            "defaults to \"%s\"",
                           robot_namespace.c_str(), odom_topic.c_str());
        } else {
            odom_topic = sdf->GetElement("odometryTopic")->Get<std::string>();
        }

        odom_frame = "odom";
        if (!sdf->HasElement("odometryFrame")) {
            ROS_WARN_NAMED("robotino_move", "RobotinoGazeboPlugin (ns = %s) missing <odometryFrame>, "
                                            "defaults to \"%s\"",
                           robot_namespace.c_str(), odom_frame.c_str());
        } else {
            odom_frame = sdf->GetElement("odometryFrame")->Get<std::string>();
        }

        base_frame = "base_link";
        if (!sdf->HasElement("robotBaseFrame")) {
            ROS_WARN_NAMED("robotino_move", "RobotinoGazeboPlugin (ns = %s) missing <robotBaseFrame>, "
                                            "defaults to \"%s\"",
                           robot_namespace.c_str(), base_frame.c_str());
        } else {
            base_frame = sdf->GetElement("robotBaseFrame")->Get<std::string>();
        }

        odom_rate = 50.0;
        if (!sdf->HasElement("odometryRate")) {
            ROS_WARN_NAMED("robotino_move", "RobotinoGazeboPlugin (ns = %s) missing <odometryRate>, "
                                            "defaults to %f",
                           robot_namespace.c_str(), odom_rate);
        } else {
            odom_rate = sdf->GetElement("odometryRate")->Get<double>();
        }

        if (!sdf->HasElement("jointName")) {
            ROS_FATAL_STREAM_NAMED("robotino_move", "RobotinoGazeboPlugin missing jointName");
            return;
        } else {
            sdf::ElementPtr element = sdf->GetElement("jointName");
            std::string joint_names = element->Get<std::string>();
            boost::erase_all(joint_names, " ");
            boost::split(joint_names_, joint_names, boost::is_any_of(","));
        }

        stddev = 0.1;// / 20 * odom_rate;
        if (!sdf->HasElement("noise")) {
            ROS_WARN_NAMED("robotino_move", "RobotinoGazeboPlugin missing noise, default: %f", stddev);
        } else {
            sdf::ElementPtr element = sdf->GetElement("noise");
            stddev = element->Get<double>();
        }

        prevUpdateTime = parent->GetWorld()->SimTime().Double();
        prev_odom_pose = parent->WorldPose();
        alive = true;

        odomCalc_pose[0] = 0;
        odomCalc_pose[1] = 0;
        odomCalc_pose[2] = 0;

        prev_w0 = 0;
        prev_w1 = 0;
        prev_w2 = 0;

        vx_noisy = 0;
        vy_noisy = 0;
        vw_noisy = 0;

        if (this->model->GetJoint("joint_x") == nullptr
            || this->model->GetJoint("joint_y") == nullptr
            || this->model->GetJoint("revolute_w") == nullptr
            || this->model->GetJoint("wheel0_joint") == nullptr
            || this->model->GetJoint("wheel1_joint") == nullptr
            || this->model->GetJoint("wheel2_joint") == nullptr
                ) {
            ROS_FATAL_STREAM_NAMED("robotino_move",
                                   "Some Joint in [joint_x, joint_y, revolute_w, wheel*_joint] not found.");
            return;
        }

        this->model->GetJoint("wheel0_joint")->SetParam("fmax", 0, 10);
        this->model->GetJoint("wheel1_joint")->SetParam("fmax", 0, 10);
        this->model->GetJoint("wheel2_joint")->SetParam("fmax", 0, 10);
        this->model->GetJoint("revolute_w")->SetParam("fmax", 0, 10);
        this->model->GetJoint("joint_x")->SetParam("fmax", 0, 10);
        this->model->GetJoint("joint_y")->SetParam("fmax", 0, 10);

        common::PID pid(4, 0.1, 0); //official 0.4, 0.01, 0 // todo maybe 40, 1 even better
        this->model->GetJointController()->SetVelocityPID(this->model->GetJoint("wheel0_joint")->GetScopedName(), pid);
        this->model->GetJointController()->SetVelocityTarget(this->model->GetJoint("wheel0_joint")->GetScopedName(),
                                                             1.0);


        this->model->GetJointController()->SetVelocityPID(this->model->GetJoint("wheel1_joint")->GetScopedName(), pid);
        this->model->GetJointController()->SetVelocityTarget(this->model->GetJoint("wheel1_joint")->GetScopedName(),
                                                             1.0);


        this->model->GetJointController()->SetVelocityPID(this->model->GetJoint("wheel2_joint")->GetScopedName(), pid);
        this->model->GetJointController()->SetVelocityTarget(this->model->GetJoint("wheel2_joint")->GetScopedName(),
                                                             1.0);

        for (const auto &joint_name : joint_names_) {
            if (joint_name != "joint_x" && joint_name != "joint_y" && joint_name != "revolute_w" &&
                joint_name != "cameraholder_link")
                joints_.push_back(this->model->GetJoint(joint_name));
        }

        // ros things..
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM_NAMED("robotino_move", "RobotinoGazeboPlugin (ns = " << robot_namespace
                                                                                  << "). A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                                                                  << "Load the Gazebo system plugin librobotino_plugin.so in the robotino_gazebo_plugin package");
            return;
        }
        ros_node.reset(new ros::NodeHandle(robot_namespace));

        tf_broadcaster.reset(new tf::TransformBroadcaster());
        prefix = tf::getPrefixParam(*ros_node);

        // cmd_vel subscriber with locker and queue
        cmd_vel_topic = "cmd_vel";
        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<geometry_msgs::Twist>(cmd_vel_topic, 1,
                                                                    boost::bind(&robotinoPlugin::cmdVelCallback, this,
                                                                                _1),
                                                                    ros::VoidPtr(), &queue);
        cmd_vel_subscriber = ros_node->subscribe(so);

        // publishers ground-truth and noise odometry
        pubGT = ros_node->advertise<nav_msgs::Odometry>("odom_gt", 1);
        pubCalc = ros_node->advertise<nav_msgs::Odometry>("odom_calc", 1);
        wheels = ros_node->advertise<geometry_msgs::TwistStamped>("wheels", 1);
        ROS_INFO_STREAM(prefix);
        joint_state_publisher_ = ros_node->advertise<sensor_msgs::JointState>("rrbot/joint_states", 100);

        // thread manager
        callback_queue = boost::thread(boost::bind(&robotinoPlugin::QueueThread, this));


        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&robotinoPlugin::OnUpdate, this, _1));
    }

    double robotinoPlugin::GaussianKernel(double mu, double sigma) {
        // gazebo ros p3d code
        // using Box-Muller transform to generate two independent standard
        // normally disbributed normal variables see wikipedia

        // normalized uniform random variable
        double U = static_cast<double>(rand_r(&this->seed)) /
                   static_cast<double>(RAND_MAX);

        // normalized uniform random variable
        double V = static_cast<double>(rand_r(&this->seed)) /
                   static_cast<double>(RAND_MAX);

        double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
        // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

        // there are 2 indep. vars, we'll just use X
        // scale to our mu and sigma
        X = sigma * X + mu;
        return X;
    }

    void
    robotinoPlugin::getVelInGlobFrame(double w0, double w1, double w2, double angle, double &vx_glob, double &vy_glob,
                                      double &vz_glob) {
        // get velocity from PID
        double v0 = w0 * R_WHEEL;
        double v1 = w1 * R_WHEEL;
        double v2 = w2 * R_WHEEL;

        // velocities of the robot in the robot frame
//        double vx = (v2 - v0) * 1 / (2 * SIN_PI_3);
//        double vy = (v0 - v1 + vx * SIN_PI_3) * 2 / 3;
        double vy = ((v0 + v2) / 2 - v1) / (1 + COS_PI_3);
        double vx = (v2 - v0) * 1 / (2 * SIN_PI_3);
        vz_glob = (v1 + vy) / R_BASE;

        // velocities in the global frame
//        angle = -angle;
        vx_glob = vx * cos(angle) - vy * sin(angle);
        vy_glob = vx * sin(angle) + vy * cos(angle);
    }

    // Called by the world update start event
    void robotinoPlugin::OnUpdate(const common::UpdateInfo &) {
        // get lock
        boost::mutex::scoped_lock scopedLock(lock);

        double currTime = this->world->SimTime().Double();
        double timestep = currTime - this->prevUpdateTime;

        // change the speed in the Robot reference frame, set linear and angular velocities

        /* -----!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!-----
         * NOTE that OUR wheels are different w.r.t. the formulation
         * that is used in the M.Sc. thesis available from Stuttgart
         * about the robotino Kinematics/Dynamics. Specifically, our
         * wheels are placed in a different order and convention.
         * This means that the transf. matrices are a bit different
         * between our and their formulation!
         * ----------------------------------------------------------*/
        pose3D = this->model->GetLink("base_link")->WorldPose();
        double w = pose3D.Rot().Yaw();
        double vx = vel_x; //vel_x * cos(w) - vel_y * sin(w);
        double vy = vel_y; //vel_x * sin(w) + vel_y * cos(w);
        double vw = vel_z;

        // apply kinematics - current wheel config.
        // odometry obtained from literature_robotino...
        // vx sin(alpha i) + vy cos(alpha i) + vw R
        double v0 = -vx * SIN_PI_3 + vy * COS_PI_3 + vw * R_BASE;
        double v1 = -vy + vw * R_BASE;
        double v2 = +vx * SIN_PI_3 + vy * COS_PI_3 + vw * R_BASE;

        // get rad/s for each wheel
        double w0 = v0 / R_WHEEL;
        double w1 = v1 / R_WHEEL;
        double w2 = v2 / R_WHEEL;

        // set velocities with PID controller for the wheels
        this->model->GetJointController()->SetVelocityTarget(this->model->GetJoint("wheel0_joint")->GetScopedName(),
                                                             w0);
        this->model->GetJointController()->SetVelocityTarget(this->model->GetJoint("wheel1_joint")->GetScopedName(),
                                                             w1);
        this->model->GetJointController()->SetVelocityTarget(this->model->GetJoint("wheel2_joint")->GetScopedName(),
                                                             w2);

        // translate those velocities applied to the joints in global frame velocities
        getVelInGlobFrame(this->model->GetJoint("wheel0_joint")->GetVelocity(0),
                          this->model->GetJoint("wheel1_joint")->GetVelocity(0),
                          this->model->GetJoint("wheel2_joint")->GetVelocity(0),
                          w, vx, vy, vw);

        this->model->GetJoint("wheel0_joint")->Update();
        this->model->GetJoint("wheel1_joint")->Update();
        this->model->GetJoint("wheel2_joint")->Update();

        // translate and rotate the model
        this->model->GetJoint("joint_x")->SetVelocity(0, vx);
        this->model->GetJoint("joint_y")->SetVelocity(0, vy);
        this->model->GetJoint("revolute_w")->SetVelocity(0, vw);


        // if rate is met publish message
        if (timestep >= 1.0 / odom_rate) {
            this->prevUpdateTime = currTime;

            // mean velocities during the timestep
            double w0_pose = this->model->GetJoint("wheel0_joint")->Position();
            double w1_pose = this->model->GetJoint("wheel1_joint")->Position();
            double w2_pose = this->model->GetJoint("wheel2_joint")->Position();

            // and add noise to avg wheel vel
            double w0_vel = (w0_pose - prev_w0) / timestep + this->GaussianKernel(0, this->stddev);
            double w1_vel = (w1_pose - prev_w1) / timestep + this->GaussianKernel(0, this->stddev);
            double w2_vel = (w2_pose - prev_w2) / timestep + this->GaussianKernel(0, this->stddev);

            // update old state with current one
            prev_w0 = w0_pose;
            prev_w1 = w1_pose;
            prev_w2 = w2_pose;

            // noisy total displacement of the wheels
            total_w0 += std::abs(w0_vel) * timestep;
            total_w1 += std::abs(w1_vel) * timestep;
            total_w2 += std::abs(w2_vel) * timestep;

            geometry_msgs::TwistStamped wheel_speed;
            wheel_speed.header.stamp = ros::Time::now();
            wheel_speed.twist.linear.x = total_w0;
            wheel_speed.twist.linear.y = total_w1;
            wheel_speed.twist.linear.z = total_w2;
            wheels.publish(wheel_speed);
            // convert in global frame [get wheel velocities and yaw and get x,y and yaw speed]
            getVelInGlobFrame(w0_vel, w1_vel, w2_vel, odomCalc_pose[2], vx_noisy, vy_noisy, vw_noisy);

            vx_noisy += this->GaussianKernel(0, this->stddev);
            vy_noisy += this->GaussianKernel(0, this->stddev);
            vw_noisy += this->GaussianKernel(0, this->stddev);

            if (isnan(vx_noisy)){
                vx_noisy = 0;
            }
            if (isnan(vy_noisy)){
                vy_noisy = 0;
            }
            if (isnan(vw_noisy)){
                vw_noisy = 0;
            }

            // get calculated position/orientation from noisy velocities
            odomCalc_pose[0] += vx_noisy * timestep;
            odomCalc_pose[1] += vy_noisy * timestep;
            odomCalc_pose[2] += vw_noisy * timestep;

            odomPublisher(vw, timestep);
        }

        // move the robot
        publishJointStates();
    }

    void robotinoPlugin::odomPublisher(double vel_z, double step_time) {
        common::Time ctime = this->world->SimTime();
        ros::Time current_time;
        current_time.sec = ctime.sec;
        current_time.nsec = ctime.nsec;
        std::string res_odom_frame = tf::resolve(prefix, odom_frame);
        std::string base_footprint_frame = tf::resolve(prefix, base_frame);

        // getting GT data for base_footprint to odom transform
/*
        tf::Quaternion qt;
        qt.setRPY(0,0,odomCalc_pose[2]);

        tf::Vector3 vt(odomCalc_pose[0], odomCalc_pose[1], 0);

        // eventually publish transform
        tf::Transform base_footprint_to_odom(qt, vt);
        tf_broadcaster->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                           current_time,
                                                           res_odom_frame,
                                                           base_footprint_frame));*/

        // publish odom ground truth topic
        odomGT.pose.pose.position.x = pose3D.Pos().X();
        odomGT.pose.pose.position.y = pose3D.Pos().Y();
        odomGT.pose.pose.position.z = pose3D.Pos().Z();
        odomGT.pose.pose.orientation.x = pose3D.Rot().X();
        odomGT.pose.pose.orientation.y = pose3D.Rot().Y();
        odomGT.pose.pose.orientation.z = pose3D.Rot().Z();
        odomGT.pose.pose.orientation.w = pose3D.Rot().W();

        // get velocity in /odom frame
        ignition::math::Vector3d linear;
        linear.X() = (pose3D.Pos().X() - prev_odom_pose.Pos().X()) / step_time;
        linear.Y() = (pose3D.Pos().Y() - prev_odom_pose.Pos().Y()) / step_time;
        if (vel_z > M_PI / step_time) {
            // we cannot calculate the angular velocity correctly
            odomGT.twist.twist.angular.z = vel_z;
        } else {
            float last_yaw = prev_odom_pose.Rot().Yaw();
            float current_yaw = pose3D.Rot().Yaw();
            while (current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
            while (current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
            float angular_diff = current_yaw - last_yaw;
            odomGT.twist.twist.angular.z = angular_diff / step_time;
        }
        prev_odom_pose = pose3D;

        // convert velocity to child_frame_id (aka base_footprint) || requested by message specification!
        float yaw = pose3D.Rot().Yaw();
        odomGT.twist.twist.linear.x = cosf(-yaw) * linear.X() - sinf(-yaw) * linear.Y();
        odomGT.twist.twist.linear.y = cosf(-yaw) * linear.Y() + sinf(-yaw) * linear.X();
        odomGT.twist.twist.linear.z = 0;

        odomGT.header.stamp = current_time;
        odomGT.header.frame_id = "world";
        odomGT.child_frame_id = base_footprint_frame;

        pubGT.publish(odomGT);

        // remind that odomCalc has noisy values and is calculated based on the wheels velocities
        // this implies that if the robot is stuck and sense movement the position will continue to grow
        odom.pose.pose.position.x = odomCalc_pose[0];
        odom.pose.pose.position.y = odomCalc_pose[1];

        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, odomCalc_pose[2]);
        myQuaternion.normalize();
        odom.pose.pose.orientation.x = myQuaternion.x();
        odom.pose.pose.orientation.y = myQuaternion.y();
        odom.pose.pose.orientation.z = myQuaternion.z();
        odom.pose.pose.orientation.w = myQuaternion.w();

        odom.twist.twist.angular.z = vw_noisy;

        if (vw_noisy > M_PI / step_time) {
            // we cannot calculate the angular velocity correctly
            odom.twist.twist.angular.z = vw_noisy;
        } else {
            double last_yaw = odomCalc_pose[2] - vw_noisy * step_time;
            double current_yaw = odomCalc_pose[2];
            while (current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
            while (current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
            double angular_diff = current_yaw - last_yaw;
            odom.twist.twist.angular.z = angular_diff / step_time;
        }

        // again convert velocity to child_frame_id (aka base_footprint)
        odom.twist.twist.linear.x = cosf(-odomCalc_pose[2]) * vx_noisy - sinf(-odomCalc_pose[2]) * vy_noisy;
        odom.twist.twist.linear.y = cosf(-odomCalc_pose[2]) * vy_noisy + sinf(-odomCalc_pose[2]) * vx_noisy;

        // set static covariance
        double gn2 = stddev * stddev;
        odom.pose.covariance[0] = gn2;
        odom.pose.covariance[7] = gn2;
        odom.pose.covariance[14] = gn2;
        odom.pose.covariance[21] = gn2;
        odom.pose.covariance[28] = gn2;
        odom.pose.covariance[35] = gn2; // todo check

        odom.twist.covariance[0] = gn2;
        odom.twist.covariance[7] = gn2;
        odom.twist.covariance[14] = gn2;
        odom.twist.covariance[21] = gn2;
        odom.twist.covariance[28] = gn2;
        odom.twist.covariance[35] = gn2; // todo check

        odom.header.stamp = current_time;
        odom.header.frame_id = res_odom_frame;
        odom.child_frame_id = base_footprint_frame;

        pubCalc.publish(odom);
    }

    // Finalize the controller
    void robotinoPlugin::FinishChild() {
        alive = false;
        queue.clear();
        queue.disable();
        ros_node->shutdown();
        callback_queue.join();
    }

    void robotinoPlugin::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg) {
        boost::mutex::scoped_lock scopedLock(lock);
        vel_x = msg->linear.x;
        vel_y = msg->linear.y;
        vel_z = msg->angular.z;
    }

    void robotinoPlugin::publishJointStates() {
        ros::Time current_time = ros::Time::now();

        joint_state_.header.stamp = current_time;
        joint_state_.name.resize(joints_.size());
        joint_state_.position.resize(joints_.size());
        joint_state_.velocity.resize(joints_.size());

        for (int i = 0; i < joints_.size(); i++) {
            physics::JointPtr joint = joints_[i];
            double velocity = joint->GetVelocity(0);
            double position = joint->Position(0);
            joint_state_.name[i] = joint->GetName();
            joint_state_.position[i] = position;
            joint_state_.velocity[i] = velocity;
        }
        joint_state_publisher_.publish(joint_state_);
    }

    GZ_REGISTER_MODEL_PLUGIN(robotinoPlugin)

}
