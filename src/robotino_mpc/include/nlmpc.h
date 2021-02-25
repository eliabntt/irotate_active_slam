//
// Created by ebonetto on 8/25/20.
//

#ifndef ROS_SLAM_NLMPC_H
#define ROS_SLAM_NLMPC_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <stdio.h>
#include "boost/bind.hpp"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <std_srvs/Empty.h>
#include "tf/tf.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <robotino_mpc/NonLinearMPCConfig.h>
#include <dynamic_reconfigure/server.h>
#include "tf/transform_listener.h"
#include "std_msgs/Int8.h"
#include "robotino_fsm_node.h"
#include "std_msgs/Float32MultiArray.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

typedef Eigen::Vector3d Position3D;
#define PI 3.14159265

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class nlmpc {
public:

    nlmpc(const ros::NodeHandle &nh_, const ros::NodeHandle &private_nh_);

    ~nlmpc();

    // Dynamic parameters
    void setPosPen(const Eigen::Vector3d &pen_pos) {
        this->pen_pos = pen_pos;
    }

    void setVelPen(const Eigen::Vector4d &pen_vel) {
        this->pen_vel = pen_vel;
    }

    void setObsPen(const Eigen::VectorXd &pen_obs) {
        this->pen_obs = pen_obs;
    }

    void setLimits(const Eigen::VectorXd &limits) {
        v_x_limit = limits(0);
        v_y_limit = limits(1);
        v_trans_limit = limits(2);
        v_yaw_rob_limit = limits(3);
        v_yaw_cam_limit = limits(4);
        v_rot_limit = limits(5);
        min_dist = limits(6);
    }

    void applyParam();

private:
    ros::NodeHandle nh_, private_nh_;

    ros::AsyncSpinner spinner;

    void initParam();

    bool isInit;

    double sampling;
    double pred_sampling;

    double v_x_limit, v_y_limit, v_trans_limit, v_yaw_rob_limit, v_yaw_cam_limit, v_rot_limit, min_dist;

    Eigen::Vector4d pen_vel;
    Eigen::Vector3d pen_pos;
    Eigen::VectorXd pen_obs;

    // solver matrices
    Eigen::Matrix<double, ACADO_NY, ACADO_NY> W;
    Eigen::Matrix<double, ACADO_NYN, ACADO_NYN> WN;
    Eigen::Matrix<double, ACADO_N + 1, ACADO_NX> state;
    Eigen::Matrix<double, ACADO_N, ACADO_NU> input;
    Eigen::Matrix<double, ACADO_N, ACADO_NY> reference;
    Eigen::Matrix<double, 1, ACADO_NYN> referenceN;
    Eigen::Matrix<double, ACADO_N + 1, ACADO_NOD> online_data;


    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MyApproxSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, nav_msgs::Odometry> MyExactSyncPolicy;
    typedef message_filters::Synchronizer<MyApproxSyncPolicy>  sync_approx;
    typedef message_filters::Synchronizer<MyExactSyncPolicy>  sync_exact;
    boost::shared_ptr<sync_approx> approx;
    boost::shared_ptr<sync_exact> exact;
    bool sim;

    bool verbose;

    ros::Time last_valid_plan_, last_oscillation_reset_;

    double solve_time_avg, cnt_failed, dist_tolerance, angle_tolerance, oscillation_timeout_, oscillation_reset_dist_, oscillation_reset_angle_, planner_patience_, max_planning_retries_;
    int reset_dist, reset_angle;
    double roll, pitch, yaw_rob, yaw_cam;

    int state_;

    bool first_odom, receivedTarget;

    double optHeadYaw;
    geometry_msgs::Pose globalRobot, globalCam;
    geometry_msgs::Pose prevRobot;
    bool need_init;
    Eigen::VectorXd prev_pen_obs;
    double prev_min_dist;

    double computeBestHead();

    void move_away_from_obstacle(const geometry_msgs::Pose &currentPose);

    void initializeSolver(Eigen::VectorXd x_0, bool first = false);

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odomRobot, const nav_msgs::Odometry::ConstPtr &odomCam);

    void getObstacles(const costmap_converter::ObstacleArrayMsg::ConstPtr &msg);

    void getTarget(const nav_msgs::Odometry::ConstPtr &waypoint);

    Eigen::VectorXd computeObsPos(const int j, const double r_x, const double r_y, const double covariance_radius,
                                  const costmap_converter::ObstacleArrayMsg &obstacleArrayMsg);

    double computeFinalYaw(double yawWaypoint);

    costmap_converter::ObstacleArrayMsg obstacles;
    Eigen::Vector3d targetPose, prevPose;
    Eigen::Vector4d nextTarget;

    std::string cmdRobotTopic, cmdCamTopic;
    std::string obstacleTopic, waypointTopic, odomRobotTopic, odomCamTopic, trajTopic, robStateTopic;
    ros::Subscriber robot_pose, cam_pose, subDest, subObst, best_heading_sub;
    std::vector<double> opt_angles;

    void getBestHead(std_msgs::Float32MultiArray msg);

    tf::StampedTransform map_to_odom;

    ros::Publisher pubOutPose, pubRobotCmd, pubCamCmd, pubPoseTraj, pubRobState;
    geometry_msgs::PoseArray selfPoseTraj;

    dynamic_reconfigure::Server<robotino_mpc::NonLinearMPCConfig> controller_dyn_config_server;

    void ControllerDynConfigCallBack(robotino_mpc::NonLinearMPCConfig &config, uint32_t level);

    boost::mutex lock;
    tf::TransformListener listener_;

    void transform(const nav_msgs::Odometry::ConstPtr &odomRobot, const nav_msgs::Odometry::ConstPtr &odomCam);

    void oscillationCheck(const Eigen::Vector3d &pos);

    void invalidTimeout();

};


#endif //ROS_SLAM_NLMPC_H
