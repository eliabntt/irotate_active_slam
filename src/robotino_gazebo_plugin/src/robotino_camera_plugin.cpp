#include <robotino_gazebo_plugin/robotino_cam_plugin.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define MATH_PI     3.1415926535897931

namespace gazebo {

    robotinoCamPlugin::robotinoCamPlugin()
            : ModelPlugin(), vel_z(0) {
        ROS_INFO("Starting robotino_camera plugin");
        this->seed = 0;
    }

    robotinoCamPlugin::~robotinoCamPlugin() {
        FinishChild();
    }


    void robotinoCamPlugin::QueueThread() {
        static const double timeout = 0.01;
        while (alive && ros_node->ok()) {
            queue.callAvailable(ros::WallDuration(timeout));
        }
    }

    void robotinoCamPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
        // Store the pointer to the model
        this->model = parent;
        this->world = parent->GetWorld();

        robot_namespace = "";
        if (!sdf->HasElement("robotNamespace")) {
            ROS_INFO_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin missing <robotNamespace>, "
                                                "defaults to \"%s\"", robot_namespace.c_str());
        } else {
            robot_namespace =
                    sdf->GetElement("robotNamespace")->Get<std::string>();
        }

        cmd_vel_topic = "/camera/cmd_vel";
        if (!sdf->HasElement("commandTopic")) {
            ROS_WARN_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin (ns = %s) missing <commandTopic>, "
                                          "defaults to \"%s\"",
                           robot_namespace.c_str(), cmd_vel_topic.c_str());
        } else {
            cmd_vel_topic = sdf->GetElement("commandTopic")->Get<std::string>();
        }

        ekf_topic = "/camera/odometry/filtered";
        if (!sdf->HasElement("commandTopic")) {
            ROS_WARN_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin (ns = %s) missing <commandTopic>, "
                                          "defaults to \"%s\"",
                           robot_namespace.c_str(), ekf_topic.c_str());
        } else {
            ekf_topic = sdf->GetElement("commandTopic")->Get<std::string>();
        }

        odom_topic = "/camera/odometry";
        if (!sdf->HasElement("odometryTopic")) {
            ROS_WARN_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin (ns = %s) missing <odometryTopic>, "
                                          "defaults to \"%s\"",
                           robot_namespace.c_str(), odom_topic.c_str());
        } else {
            odom_topic = sdf->GetElement("odometryTopic")->Get<std::string>();
        }

        odom_frame = "base_link";
        if (!sdf->HasElement("odometryFrame")) {
            ROS_WARN_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin (ns = %s) missing <odometryFrame>, "
                                          "defaults to \"%s\"",
                           robot_namespace.c_str(), odom_frame.c_str());
        } else {
            odom_frame = sdf->GetElement("odometryFrame")->Get<std::string>();
        }

        base_frame = "cameraholder_link";
        if (!sdf->HasElement("robotBaseFrame")) {
            ROS_WARN_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin (ns = %s) missing <robotBaseFrame>, "
                                          "defaults to \"%s\"",
                           robot_namespace.c_str(), base_frame.c_str());
        } else {
            base_frame = sdf->GetElement("robotBaseFrame")->Get<std::string>();
        }

        odom_rate = 50.0;
        if (!sdf->HasElement("odometryRate")) {
            ROS_WARN_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin (ns = %s) missing <odometryRate>, "
                                          "defaults to %f",
                           robot_namespace.c_str(), odom_rate);
        } else {
            odom_rate = sdf->GetElement("odometryRate")->Get<double>();
        }

        stddev = 0.01;// / 20 * odom_rate;
        if (!sdf->HasElement("noise")) {
            ROS_WARN_NAMED("camera_rotation","RobotinoCameraGazeboPlugin Plugin missing noise, default: %f", stddev);
        } else {
            sdf::ElementPtr element = sdf->GetElement("noise");
            stddev = element->Get<double>();
        }

        prevUpdateTime = parent->GetWorld()->SimTime().Double();
        alive = true;

        odomCalc_pose = 0;

        prev_w0 = 0;

        if (this->model->GetJoint("cameraholder_joint") == nullptr) {
            ROS_FATAL_STREAM_NAMED("camera_rotation",
                                   "[cameraholder_joint] not found.");
            return;
        }

        this->model->GetJoint("cameraholder_joint")->SetParam("fmax", 0, 10);
        this->model->GetJoint("cameraholder_joint")->SetParam("vel", 0, 3);

        p = 0.9;
        i = 7;
        d = 0.0;
        integral = 0;
        pre_error = 0;
        dt = 0.01;
        max_v = 1;
        min_v = -1;
        if (!sdf->HasElement("p")) {
            ROS_WARN_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin (ns = %s) missing <p>, "
                                          "defaults to \"%f\"",
                           robot_namespace.c_str(), p);
        } else {
            p = sdf->GetElement("p")->Get<double>();
        }
        if (!sdf->HasElement("i")) {
            ROS_WARN_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin (ns = %s) missing <i>, "
                                          "defaults to \"%f\"",
                           robot_namespace.c_str(), i);
        } else {
            i = sdf->GetElement("i")->Get<double>();
        }
        if (!sdf->HasElement("d")) {
            ROS_WARN_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin (ns = %s) missing <d>, "
                                          "defaults to \"%f\"",
                           robot_namespace.c_str(), d);
        } else {
            d = sdf->GetElement("commandTopic")->Get<double>();
        }
        if (!sdf->HasElement("dt")) {
            ROS_WARN_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin (ns = %s) missing <dt>, "
                                       "defaults to \"%f\"",
                           robot_namespace.c_str(), dt);
        } else {
            dt = sdf->GetElement("dt")->Get<double>();
        }
        if (!sdf->HasElement("max_v")) {
            ROS_WARN_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin (ns = %s) missing <max_v>, "
                                       "defaults to \"%f\"",
                           robot_namespace.c_str(), max_v);
        } else {
            max_v = sdf->GetElement("max_v")->Get<double>();
        }
        if (!sdf->HasElement("min_v")) {
            ROS_WARN_NAMED("camera_rotation", "RobotinoCameraGazeboPlugin (ns = %s) missing <min_v>, "
                                       "defaults to \"%f\"",
                           robot_namespace.c_str(), min_v);
        } else {
            min_v = sdf->GetElement("min_v")->Get<double>();
        }



        for (const auto &joint_name : joint_names_) {
                joints_.push_back(this->model->GetJoint(joint_name));
        }

        // ros things..
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM_NAMED("camera_rotation", "RobotinoGazeboPlugin (ns = " << robot_namespace
                                                                                   << "). A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                                                                   << "Load the Gazebo system plugin librobotino_plugin.so in the robotino_gazebo_plugin package");
            return;
        }
        ros_node.reset(new ros::NodeHandle(robot_namespace));

        // cmd_vel subscriber with locker and queue
        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<geometry_msgs::Twist>(cmd_vel_topic, 1,
                                                                    boost::bind(&robotinoCamPlugin::cmdVelCallback, this,
                                                                                _1),
                                                                    ros::VoidPtr(), &queue);
        cmd_vel_subscriber = ros_node->subscribe(so);

        // ekf subscriber with locker and queue
        ekf_topic = "/camera/odometry/filtered";
        ros::SubscribeOptions sekf =
                ros::SubscribeOptions::create<nav_msgs::Odometry>(ekf_topic, 1,
                                                                    boost::bind(&robotinoCamPlugin::ekfCallback, this,
                                                                                _1),
                                                                    ros::VoidPtr(), &queue);
        ekf_subscriber = ros_node->subscribe(sekf);


        // publishers ground-truth and noise odometry
        pubCalc = ros_node->advertise<nav_msgs::Odometry>("/camera/odometry", 1);
        pubMotor = ros_node->advertise<std_msgs::Float32>("/pid/out", 1);
        logCamera = ros_node->advertise<geometry_msgs::TwistStamped>("camera_rot", 1);
        total_w = 0;

        // thread manager
        callback_queue = boost::thread(boost::bind(&robotinoCamPlugin::QueueThread, this));


        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&robotinoCamPlugin::OnUpdate, this, _1));
    }

    double robotinoCamPlugin::CalculatePID(double goal, double current){
        double err = goal - current;
  //      std::cout << "err " << err << std::endl;

        double p_term = p * err;

        integral += err * dt;
        double i_term = i * integral;

        double der = (err - pre_error) / dt;
        double d_term = d * der;

        double out = p_term + i_term + d_term;

        out = (out > max_v) ? max_v : out;
        out = (out < min_v) ? min_v : out;

        pre_error = err;
     //   std::cout << "out controller" << out << std::endl;
        std_msgs::Float32 a;
        a.data = out;
        pubMotor.publish(a);
        return out;
    }

    double robotinoCamPlugin::GaussianKernel(double mu, double sigma)
    {
        // gazebo ros p3d code
        // using Box-Muller transform to generate two independent standard
        // normally disbributed normal variables see wikipedia

        // normalized uniform random variable
        double U = static_cast<double>(rand_r(&this->seed)) /
                   static_cast<double>(RAND_MAX);

        // normalized uniform random variable
        double V = static_cast<double>(rand_r(&this->seed)) /
                   static_cast<double>(RAND_MAX);

        double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
        // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

        // there are 2 indep. vars, we'll just use X
        // scale to our mu and sigma
        X = sigma * X + mu;
        return X;
    }

    // Called by the world update start event
    void robotinoCamPlugin::OnUpdate(const common::UpdateInfo &) {
        // get lock
        boost::mutex::scoped_lock scopedLock(lock);

        double vw = vel_z;  // todo force to zero or control camera? or add camera control

        // fixme get rad/s for camera
        double v0 = vw;

        double set_vel = 0;

        if (v0 != 0)
            set_vel = CalculatePID(v0, ekf_vel);

        this->model->GetJoint("cameraholder_joint")->SetVelocity(0, set_vel);

        double currTime = this->world->SimTime().Double();
        double timestep = currTime - this->prevUpdateTime;


        // if rate is met publish message
        if (timestep >= 1.0 / odom_rate) {
            this->prevUpdateTime = currTime;

            // todo mean velocities during the timestep
            double w0_pose = this->model->GetJoint("cameraholder_joint")->Position();

            // and add noise
            double w0_vel = (w0_pose - prev_w0) / timestep + this->GaussianKernel(0, this->stddev);

            total_w += abs(w0_vel) * timestep;
            geometry_msgs::TwistStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.twist.linear.x = total_w;
            logCamera.publish(msg);

            // update old state with current one
            prev_w0 = w0_pose;

            // get calculated position/orientation from noisy velocities
            odomCalc_pose += w0_vel * timestep;

            odomPublisher(w0_vel, timestep);
        }

    }

    void robotinoCamPlugin::odomPublisher(double vel_z, double step_time) {
        common::Time ctime = world->SimTime();
        ros::Time current_time;
        current_time.sec = ctime.sec;
        current_time.nsec = ctime.nsec;

        // remind that odomCalc has noisy values and is calculated based on the wheels velocities
        // this implies that if the robot is stuck and sense movement the position will continue to grow
        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        odom.pose.pose.position.z = 0.0; //fixme hardcoded

        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, odomCalc_pose);
        myQuaternion.normalize();
        odom.pose.pose.orientation.x = myQuaternion.x();
        odom.pose.pose.orientation.y = myQuaternion.y();
        odom.pose.pose.orientation.z = myQuaternion.z();
        odom.pose.pose.orientation.w = myQuaternion.w();

        odom.twist.twist.angular.z = vel_z;

        if (vel_z > M_PI / step_time) {
            // we cannot calculate the angular velocity correctly
            odom.twist.twist.angular.z = vel_z;
        } else {
            double last_yaw = odomCalc_pose - vel_z * step_time;
            double current_yaw = odomCalc_pose;
            while (current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
            while (current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
            double angular_diff = current_yaw - last_yaw;
            odom.twist.twist.angular.z = angular_diff / step_time;
        }

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
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = base_frame;

        pubCalc.publish(odom);
    }

    // Finalize the controller
    void robotinoCamPlugin::FinishChild() {
        alive = false;
        queue.clear();
        queue.disable();
        ros_node->shutdown();
        callback_queue.join();
    }

    void robotinoCamPlugin::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg) {
        boost::mutex::scoped_lock scopedLock(lock);
        vel_z = msg->angular.z;

        vel_z = (vel_z > max_v) ? max_v : vel_z;
        vel_z = (vel_z < min_v) ? min_v : vel_z;
    }

    void robotinoCamPlugin::ekfCallback(const nav_msgs::OdometryConstPtr &msg) {
        boost::mutex::scoped_lock scopedLock(lock);
        ekf_vel = msg->twist.twist.angular.z;
    }

    GZ_REGISTER_MODEL_PLUGIN(robotinoCamPlugin)
}
