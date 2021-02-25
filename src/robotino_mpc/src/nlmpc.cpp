//
// Created by ebonetto on 8/25/20.
//

#include "nlmpc.h"

void getRPY(geometry_msgs::Pose odom, double &roll, double &pitch, double &yaw) {
    tf::Quaternion q(
            odom.orientation.x,
            odom.orientation.y,
            odom.orientation.z,
            odom.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}


nlmpc::nlmpc(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh) :
        nh_(nh),
        private_nh_(private_nh),
        isInit(false),
        verbose(false),
        solve_time_avg(0),
        first_odom(true),
        need_init(true),
        controller_dyn_config_server(nh),
        spinner(2) {
    acado_initializeSolver();

    W.setZero();
    WN.setZero();
    input.setZero();
    state.setZero();
    reference.setZero();
    referenceN.setZero();

    dynamic_reconfigure::Server<robotino_mpc::NonLinearMPCConfig>::CallbackType f_controller;
    f_controller = boost::bind(&nlmpc::ControllerDynConfigCallBack, this, _1, _2);
    controller_dyn_config_server.setCallback(f_controller);

    initParam();
    applyParam();

    pubRobotCmd = nh_.advertise<geometry_msgs::Twist>(cmdRobotTopic, 1);
    pubCamCmd = nh_.advertise<geometry_msgs::Twist>(cmdCamTopic, 1);
    pubRobState = nh_.advertise<std_msgs::Int8>(robStateTopic, 1);

    // debug
    pubPoseTraj = nh_.advertise<geometry_msgs::PoseArray>(trajTopic, 1);

    subObst = nh_.subscribe<costmap_converter::ObstacleArrayMsg>(obstacleTopic, 10,
                                                                 boost::bind(&nlmpc::getObstacles, this, _1));
    subDest = nh_.subscribe<nav_msgs::Odometry>(waypointTopic, 10, boost::bind(&nlmpc::getTarget, this, _1));

    best_heading_sub = nh_.subscribe<std_msgs::Float32MultiArray>("/opt_head", 1, &nlmpc::getBestHead, this);

    message_filters::Subscriber<nav_msgs::Odometry> odomRobotSub(nh_, odomRobotTopic, 1);
    message_filters::Subscriber<nav_msgs::Odometry> odomCamSub(nh_, odomCamTopic, 1);
    if (sim) {
        exact.reset(new sync_exact(MyExactSyncPolicy (10), odomRobotSub, odomCamSub));
        exact->registerCallback(boost::bind(&nlmpc::odomCallback,this, _1, _2));
    }
    else{
        approx.reset(new sync_approx(MyApproxSyncPolicy (10), odomRobotSub, odomCamSub));
        approx->registerCallback(boost::bind(&nlmpc::odomCallback,this, _1, _2));
    }


    //  spinner.start();
    ros::spin();
    while (ros::ok()) {}
//    spinner.stop();
}

void nlmpc::getBestHead(std_msgs::Float32MultiArray msg) {
    if (msg.data.empty()) {
        opt_angles = std::vector<double>{};
    } else {
        opt_angles = std::vector<double>(msg.data.begin(), msg.data.end());
    }
}

nlmpc::~nlmpc() {}

void nlmpc::getObstacles(const costmap_converter::ObstacleArrayMsg::ConstPtr &msg) {
    if (msg != nullptr)
        if (msg->header.stamp > obstacles.header.stamp + ros::Duration(0.3))
            obstacles = *msg;
}

void nlmpc::move_away_from_obstacle(const geometry_msgs::Pose &currentPose) {
    ROS_ERROR_STREAM("MOVE AWAY");
    Eigen::MatrixXd obs = online_data.block(0, 0, 1, ACADO_NOD); // get the first 10 nearest obstacles
    double x = 0;
    double y = 0;
    double cum_weight = 0;
    for (int i = 0; i < ACADO_NOD / 2; i++) {
        double dist = std::sqrt((obs.coeff(i) - currentPose.position.x) * (obs.coeff(i) - currentPose.position.x) +
                                (obs.coeff(i + ACADO_NOD / 2) - currentPose.position.y) *
                                (obs.coeff(i + ACADO_NOD / 2) - currentPose.position.y));
        x += obs.coeff(i) * std::exp(1 / (dist + 0.00001));
        y += obs.coeff(i + ACADO_NOD / 2) * std::exp(1 / (dist + 0.00001));
        cum_weight += std::exp(1 / (dist + 0.00001));
    }
    x /= cum_weight;
    y /= cum_weight;
    double angle = std::atan2(currentPose.position.y - y, currentPose.position.x - x);

    geometry_msgs::Twist msg;
    msg.linear.x = (0.2 * cos(yaw_rob) - 0.2 * std::sin(yaw_rob)) * std::cos(angle);
    msg.linear.y = (0.2 * std::sin(yaw_rob) + 0.2 * std::cos(yaw_rob)) * std::sin(angle);

    pubRobotCmd.publish(msg);
    pubRobotCmd.publish(msg);
    pubRobotCmd.publish(msg);
    pubRobotCmd.publish(msg);
    ROS_INFO_STREAM("msg linear " << msg.linear.x << " " << msg.linear.y);
    ros::Duration(1).sleep();
}

void nlmpc::getTarget(const nav_msgs::Odometry::ConstPtr &waypoint) {
    receivedTarget = true;
    last_oscillation_reset_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();

    state_ = robotino_fsm_node::RECEIVED;
    if (waypoint->header.frame_id == "emergency") {
        move_away_from_obstacle(waypoint->pose.pose);
        geometry_msgs::Twist msg;
        pubRobotCmd.publish(msg);
        targetPose(0) = globalRobot.position.x;
        targetPose(1) = globalRobot.position.y;
        state_ = robotino_fsm_node::WAITING;
        receivedTarget = false;
        need_init = true;
        ros::Duration(14).sleep();
        pubRobotCmd.publish(msg);
        return;
    }

    targetPose(0) = waypoint->pose.pose.position.x;
    targetPose(1) = waypoint->pose.pose.position.y;
    double r, p, y;

    getRPY(waypoint->pose.pose, r, p, y);

    double y2 = y > 0 ? std::fmod(y - 2 * M_PI, 2 * M_PI) : std::fmod(y + 2 * M_PI, 2 * M_PI);

    // nearest angle
    std::abs(y2 - std::fmod((yaw_cam + yaw_rob), 2 * M_PI)) < std::abs(y - std::fmod((yaw_cam + yaw_rob), 2 * M_PI))
    ? targetPose(2) = y2 : targetPose(2) = y; //general

    nextTarget(0) = waypoint->twist.twist.linear.x;
    nextTarget(1) = waypoint->twist.twist.linear.y;
    nextTarget(2) = waypoint->twist.twist.linear.z;
    nextTarget(3) = waypoint->twist.twist.angular.z; //cam
}

void nlmpc::initParam() {
    private_nh_.param<bool>("verbose", verbose, false);

    receivedTarget = false;
    if (first_odom)
        state_ = robotino_fsm_node::WAITING;
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();

    if (!private_nh_.getParam("sampling_time", sampling)) {
        ROS_ERROR("no sampling_time");
        abort();
    }
    if (!private_nh_.getParam("pred_sampling_time", pred_sampling)) {
        ROS_ERROR("no pred_sampling_time");
        abort();
    }
    if (!private_nh_.getParam("odom_robot", odomRobotTopic)) {
        ROS_ERROR("no odom_robot topic");
        abort();
    }
    if (!private_nh_.getParam("odom_cam", odomCamTopic)) {
        ROS_ERROR("no odom_cam topic");
        abort();
    }
    if (!private_nh_.getParam("dest", waypointTopic)) {
        ROS_ERROR("no dest");
        abort();
    }
    if (!private_nh_.getParam("obstacles", obstacleTopic)) {
        ROS_ERROR("no obstacles");
        abort();
    }
    if (!private_nh_.getParam("cmd_vel_robot", cmdRobotTopic)) {
        ROS_ERROR("no cmd_vel_robot");
        abort();
    }
    if (!private_nh_.getParam("cmd_vel_cam", cmdCamTopic)) {
        ROS_ERROR("no cmd_vel_cam");
        abort();
    }
    if (!private_nh_.getParam("rob_state", robStateTopic)) {
        ROS_ERROR("no rob_state");
        abort();
    }
    if (!private_nh_.getParam("traj", trajTopic)) {
        ROS_ERROR("no traj");
        abort();
    }
    if (!private_nh_.getParam("sim", sim)) {
        ROS_ERROR("no sim");
        abort();
    }

    for (int i = 0; i < ACADO_N + 1; i++) {
        Eigen::VectorXd tmp(ACADO_NOD);
        tmp.setZero();
        online_data.block(i, 0, 1, ACADO_NOD) = tmp.transpose().matrix();
    }

    Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N + 1 >>(const_cast<double *>(acadoVariables.od)) =
            online_data.transpose();

    if (verbose) {
        std::cout << "acado online data: " << std::endl << online_data << std::endl;
    }

    isInit = true;
    reset_dist = 0;
    reset_angle = 0;
    ROS_INFO("Nonlinear MPC: initialized correctly");
}

void nlmpc::oscillationCheck(const Eigen::Vector3d &pos) {
    double x_diff = pos[0] - prevPose[0];
    double y_diff = pos[1] - prevPose[1];
    double sq_dist = x_diff * x_diff + y_diff * y_diff;

    double th_diff = std::fmod(pos[2] - prevPose[2], 2 * PI);

    // timeout == every x s I need to check
    // < == wall time has passed
    if (oscillation_timeout_ > 0.0 &&
        last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now()) {

        prevPose = pos;

        //if we've moved far enough... we can reset our flags
        if ((sq_dist >= oscillation_reset_dist_ * oscillation_reset_dist_ ||
             std::fabs(th_diff) > oscillation_reset_angle_) && abs(reset_dist - reset_angle) < 5) {
            if (sq_dist < oscillation_reset_dist_ * oscillation_reset_dist_) reset_dist += 1;
            if (std::fabs(th_diff) < oscillation_reset_angle_) reset_angle += 1;
            if (sq_dist > 1) {
                geometry_msgs::Twist msg;
                pubCamCmd.publish(msg);
                pubRobotCmd.publish(msg);
                ROS_WARN_STREAM("BIG ODOM CHANGE!");
                need_init = true;
                ros::Duration(3).sleep();
            }
            if (prev_min_dist != -1) {
                setObsPen(prev_pen_obs);
                min_dist = prev_min_dist;
                prev_min_dist = -1;
                applyParam();
            }
            last_oscillation_reset_ = ros::Time::now();
        } else {        // if goal is nearby check if reached
            if (std::pow((targetPose(0) - pos(0)) * (targetPose(0) - pos(0)) +
                         (targetPose(1) - pos(1)) * (targetPose(1) - pos(1)), 0.5) < dist_tolerance * 2 &&
                ((nextTarget(2) == -1 &&
                  std::fabs(std::fmod(yaw_rob + yaw_rob - targetPose(2), 2 * PI)) < angle_tolerance) ||
                 nextTarget(2) == 0)) {
                state_ = robotino_fsm_node::REACHED;
                reset_dist = 0;
                reset_angle = 0;
                ROS_INFO_STREAM("TARGET REACHED");
            } else {
                reset_dist = 0;
                reset_angle = 0;
                state_ = robotino_fsm_node::OSCILLATING;
            }
        }
    }
}

void nlmpc::invalidTimeout() {
    // attempt end == timeout
    ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

    if (planner_patience_ > 0 && (ros::Time::now() > attempt_end || cnt_failed > uint32_t(max_planning_retries_))) {
        if (std::pow((targetPose(0) - globalRobot.position.x) * (targetPose(0) - globalRobot.position.x) +
                     (targetPose(1) - globalRobot.position.y) * (targetPose(1) - globalRobot.position.y), 0.5) <
            2 * dist_tolerance && ((nextTarget(2) == -1 &&
                                    std::fabs(std::fmod(yaw_cam + yaw_rob - targetPose(2), 2 * PI)) <
                                    angle_tolerance) ||
                                   nextTarget(2) == 0)) {
            state_ = robotino_fsm_node::REACHED;
            ROS_INFO_STREAM("TARGET REACHED");
            reset_dist = 0;
            reset_angle = 0;
            return;
        }

        state_ = robotino_fsm_node::INVALID;
        if (prev_min_dist == -1) {
            prev_min_dist = min_dist;
            prev_pen_obs = pen_obs;
        }

        min_dist = prev_min_dist - prev_min_dist * 0.1;
        Eigen::VectorXd pen_obs(10);
        pen_obs.setConstant(prev_pen_obs(0) - 1);
        setObsPen(pen_obs);

        applyParam();

        reset_dist = 0;
        reset_angle = 0;
    }

}

void nlmpc::applyParam() {
    W.block(0, 0, 4, 4) = pen_vel.asDiagonal(); // absolute x and y veocities +
    W.block(4, 4, 3, 3) = pen_pos.asDiagonal();
    W.block(7, 7, 10, 10) = pen_obs.asDiagonal();
    WN.block(0, 0, 3, 3) = pen_pos.asDiagonal();

    Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_NY >>(const_cast<double *>(acadoVariables.W)) = W.transpose();
    Eigen::Map<Eigen::Matrix<double, ACADO_NYN, ACADO_NYN >>(const_cast<double *>(acadoVariables.WN)) =
            WN.transpose();

    for (size_t i = 0; i < ACADO_N; ++i) {
        acadoVariables.lbValues[4 * i] = -v_x_limit;       // min x vel
        acadoVariables.lbValues[4 * i + 1] = -v_y_limit;  // min y vel
        acadoVariables.lbValues[4 * i + 2] = -v_yaw_rob_limit;    // min yaw rob
        acadoVariables.lbValues[4 * i + 3] = -v_yaw_cam_limit;    // min yaw cam
        acadoVariables.ubValues[4 * i] = v_x_limit;        // max x vel
        acadoVariables.ubValues[4 * i + 1] = v_y_limit;   // max y vel
        acadoVariables.ubValues[4 * i + 2] = v_yaw_rob_limit;    // max yaw rob
        acadoVariables.ubValues[4 * i + 3] = v_yaw_cam_limit;    // max yaw cam
    }

    for (int i = 0; i < ACADO_N; i++) {
        acadoVariables.ubAValues[12 * i] = v_trans_limit * 100;    // max trans rob
        acadoVariables.lbAValues[12 * i] = 0;    // min trans rob
        acadoVariables.ubAValues[12 * i + 1] = v_rot_limit;    // max rot rob
        acadoVariables.lbAValues[12 * i + 1] = 0;    // min rot rob
        acadoVariables.lbAValues[12 * i + 2] = min_dist;
        acadoVariables.lbAValues[12 * i + 3] = min_dist;
        acadoVariables.lbAValues[12 * i + 4] = min_dist;
        acadoVariables.lbAValues[12 * i + 5] = min_dist;
        acadoVariables.lbAValues[12 * i + 6] = min_dist;
        acadoVariables.lbAValues[12 * i + 7] = min_dist;
        acadoVariables.lbAValues[12 * i + 8] = min_dist;
        acadoVariables.lbAValues[12 * i + 9] = min_dist;
        acadoVariables.lbAValues[12 * i + 10] = min_dist;
        acadoVariables.lbAValues[12 * i + 11] = min_dist;
    }

    if (verbose) {
        std::cout << "pen_pos: " << pen_pos.transpose() << std::endl;
        std::cout << "pen_vel: " << pen_vel.transpose() << std::endl;
        std::cout << "WN = \n" << WN << std::endl;
        std::cout << "W = \n" << W << std::endl;
        std::cout << "v_x_limit " << v_x_limit << std::endl;       // min x vel
        std::cout << "v_y_limit " << v_y_limit << std::endl;  // min y vel
        std::cout << "v_trans_limit " << v_trans_limit << std::endl;    // min trans rob
        std::cout << "v_yaw_rob_limit " << v_yaw_rob_limit << std::endl;    // min yaw rob
        std::cout << "v_yaw_cam_limit " << v_yaw_cam_limit << std::endl;    // min yaw cam
        std::cout << "v_rot_limit " << v_rot_limit << std::endl;    // min yaw tot
        std::cout << "min_dist " << min_dist << std::endl;    // min yaw tot
    }
}

void nlmpc::transform(const nav_msgs::Odometry::ConstPtr &odomRobot, const nav_msgs::Odometry::ConstPtr &odomCam) {
    tf::Transform odom_to_target;
    tf::poseMsgToTF(odomRobot->pose.pose, odom_to_target);
    try {
        listener_.lookupTransform("/map", "odom", ros::Time::now(), map_to_odom);
    }
    catch (tf2::TransformException &e) {
        ROS_WARN_STREAM("using old tf");
    }
    tf::poseTFToMsg(map_to_odom * odom_to_target, globalRobot);
    tf::poseMsgToTF(odomCam->pose.pose, odom_to_target);
    tf::poseTFToMsg(map_to_odom * odom_to_target, globalCam);
}

double nlmpc::computeFinalYaw(double opt_angle) {
    double x_mpc = nextTarget(2) == -1 ? targetPose(0) : nextTarget(0);
    double y_mpc = nextTarget(2) == -1 ? targetPose(1) : nextTarget(1);

    double target_angle = nextTarget(2) == -1 ? targetPose(2) : nextTarget(3);

    double lin_dist = std::pow((x_mpc - globalRobot.position.x) * (x_mpc - globalRobot.position.x) +
                               (y_mpc - globalRobot.position.y) * (y_mpc - globalRobot.position.y), 0.5);

    double final_angle_1 = (target_angle * std::exp(-6 * lin_dist) + opt_angle * std::exp(-0.5 / lin_dist)) /
                           (std::exp(-6 * lin_dist) + std::exp(-0.5 / lin_dist));
    double final_angle_2 = (target_angle * std::exp(-6 * lin_dist) +
                            (opt_angle > 0 ? opt_angle - 2 * M_PI : opt_angle + 2 * M_PI) * std::exp(-0.5 / lin_dist)) /
                           (std::exp(-6 * lin_dist) + std::exp(-0.5 / lin_dist));
    double dist_1_target = std::fmod((target_angle - final_angle_1 + M_PI + 2 * M_PI), (2 * M_PI)) - M_PI;
    double dist_2_target = std::fmod((target_angle - final_angle_2 + M_PI + 2 * M_PI), (2 * M_PI)) - M_PI;

    double final_angle = (abs(dist_1_target) < abs(dist_2_target) ? final_angle_1 : final_angle_2);

    return final_angle;
}

double nlmpc::computeBestHead() {
    if (!opt_angles.empty()) {
        double dist = 2 * M_PI;
        double angle = 0;
        for (auto i: opt_angles) {
            i = i * M_PI / 180;
            if (std::fmod((yaw_rob + yaw_cam - i + M_PI + 2 * M_PI), (2 * M_PI)) - M_PI < dist) {
                dist = std::fmod((yaw_rob + yaw_cam - i + M_PI + 2 * M_PI), (2 * M_PI)) - M_PI;
                angle = i;
            }
        }
        return angle;
    } else {
        return nextTarget(2) == -1 ? targetPose(2) : nextTarget(3);
    }
}

void nlmpc::odomCallback(const nav_msgs::Odometry::ConstPtr &odomRobot, const nav_msgs::Odometry::ConstPtr &odomCam) {
    boost::mutex::scoped_lock scopedLock(this->lock);

    geometry_msgs::Twist cmd;
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;

    try {
        transform(odomRobot, odomCam);
    }
    catch (tf::TransformException &e) {
        ROS_WARN_STREAM(e.what());
        pubRobotCmd.publish(cmd);
        pubCamCmd.publish(cmd);
        return;
    }

    getRPY(globalRobot, roll, pitch, yaw_rob);
    getRPY(globalCam, roll, pitch, yaw_cam);

    optHeadYaw = computeBestHead();
    double targetYaw = receivedTarget ? computeFinalYaw(optHeadYaw) : targetPose(2);
    double target_db = nextTarget(2) == -1 ? targetPose(2) : nextTarget(3);
    double x_mpc = nextTarget(2) == -1 ? targetPose(0) : nextTarget(0);
    double y_mpc = nextTarget(2) == -1 ? targetPose(1) : nextTarget(1);

    ROS_INFO_STREAM(
            "opt " << (optHeadYaw > 0 ? optHeadYaw * 180 / M_PI : (optHeadYaw + 2 * M_PI) * 180 / M_PI) << " wanted "
                   << (target_db > 0 ? target_db * 180 / M_PI : (target_db + 2 * M_PI) * 180 / M_PI)
                   << " sought "
                   << (targetYaw > 0 ? targetYaw * 180 / M_PI : (targetYaw + 2 * M_PI) * 180 / M_PI)
                   << " dist "
                   << nextTarget(2)
                   << " " << std::pow((x_mpc - globalRobot.position.x) * (x_mpc - globalRobot.position.x) +
                                      (y_mpc - globalRobot.position.y) * (y_mpc - globalRobot.position.y), 0.5)
                   << " current "
                   << ((yaw_rob + yaw_cam) > 0 ? (yaw_rob + yaw_cam) * 180 / M_PI : ((yaw_rob + yaw_cam) + 2 * M_PI) *
                                                                                    180 / M_PI));
//
//    ROS_INFO_STREAM(nextTarget(2) << " " << nextTarget(3));

// get nearest positive/negative yaw to be sure shortest path is chosen
    if (std::abs((yaw_rob + yaw_cam) - targetYaw) > M_PI) {
        yaw_rob > 0 ? yaw_rob -= 2 * M_PI : yaw_rob += 2 * M_PI;
        yaw_cam > 0 ? yaw_cam -= 2 * M_PI : yaw_cam += 2 * M_PI;
    }

    double l1 = (odomRobot->pose.covariance[0] + odomRobot->pose.covariance[7]) / 2 +
                sqrt(pow((odomRobot->pose.covariance[0] - odomRobot->pose.covariance[7]) / 2, 2) +
                     pow(odomRobot->pose.covariance[1], 2));
    double l2 = (odomRobot->pose.covariance[0] + odomRobot->pose.covariance[7]) / 2 -
                sqrt(pow((odomRobot->pose.covariance[0] - odomRobot->pose.covariance[7]) / 2, 2) +
                     pow(odomRobot->pose.covariance[1], 2));
    double theta = (odomRobot->pose.covariance[0] == 0 ? (odomRobot->pose.covariance[0] > odomRobot->pose.covariance[7]
                                                          ? 0 : M_PI / 2) : std::atan2(
            l1 - odomRobot->pose.covariance[0], odomRobot->pose.covariance[7]));
    double covariance_radius = std::max(
            {sqrt(l1) * cos(theta), sqrt(l2) * sin(theta), sqrt(l2) * cos(theta), sqrt(l1) * sin(theta)});

    Eigen::VectorXd x_0(ACADO_NX);
    x_0 << globalRobot.position.x, globalRobot.position.y, yaw_rob, std::fmod((yaw_rob + yaw_cam),
                                                                              2 * PI), 0; // 0 is the dummy variable

    if (first_odom || need_init) {
        need_init = false;
        initializeSolver(x_0, first_odom);
        first_odom = false;
    }

    ros::WallTime start_time = ros::WallTime::now();

    if (!receivedTarget) {
        targetPose << x_0(0), x_0(1), x_0(3);
        // no need to do computation
        pubRobotCmd.publish(cmd);
        pubCamCmd.publish(cmd);
        state_ = robotino_fsm_node::WAITING;
        return;
    } else {
        // if goal is not just being received! [just safety check + debug]
        std_msgs::Int8 robState;

        if (state_ != robotino_fsm_node::RECEIVED) {
            // if I'm trying to reach a target check for oscillation or timeout
            oscillationCheck(Eigen::Vector3d(x_0(0), x_0(1), x_0(3)));
            invalidTimeout();

            // if goal is nearby check if reached
            if (std::pow((targetPose(0) - x_0(0)) * (targetPose(0) - x_0(0)) +
                         (targetPose(1) - x_0(1)) * (targetPose(1) - x_0(1)), 0.5) < dist_tolerance) {
                if ((nextTarget(2) == -1 && std::fabs(std::fmod(x_0(3) - targetPose(2), 2 * PI)) < angle_tolerance) ||
                    nextTarget(2) == 0) {
                    state_ = robotino_fsm_node::REACHED;
                    ROS_INFO_STREAM("TARGET REACHED");
                    reset_dist = 0;
                    reset_angle = 0;
                }
            }

            if (state_ != robotino_fsm_node::OPERATING || need_init) {
                if (state_ == robotino_fsm_node::REACHED) {
                    ROS_INFO_STREAM("Goal Reached");
                } else {
                    reset_dist = 0;
                    reset_angle = 0;
                    ROS_WARN_STREAM("Recovery behaviour requested because " << state_);
                }

                // be sure that robot stops now and on next iterations. Override commands and flags
                targetPose << x_0(0), x_0(1), x_0(3);
                receivedTarget = false;

                // cmd is still all zeroes from before
                // [either reached the target or will wait for another plan to recovery]
                pubRobotCmd.publish(cmd);
                pubCamCmd.publish(cmd);

                // publish state
                robState.data = state_;
                pubRobState.publish(robState);

                return;
            }
        }
        robState.data = state_;
        pubRobState.publish(robState);
        state_ = robotino_fsm_node::OPERATING;
    }

    costmap_converter::ObstacleArrayMsg current_obstacles = obstacles;

    for (size_t i = 0; i < ACADO_N; i++) {
//        ROS_INFO_STREAM("---------------- " << i << " -----------------");
        Eigen::VectorXd tmp = computeObsPos(i, x_0(0), x_0(1), covariance_radius, current_obstacles);
        online_data.block(i, 0, 1, ACADO_NOD) << tmp.transpose().matrix();
        reference.block(i, 0, 1, ACADO_NY)
                << 0, 0, 0, 0, targetPose(0), targetPose(1), targetYaw, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        // vel_{x,y,y_rob,y_cam}, final_orientation, obstacles potential{1-10}
    }
    referenceN << targetPose(0), targetPose(1), targetYaw;

    Eigen::VectorXd tmp = computeObsPos(ACADO_N, x_0(0), x_0(1), covariance_radius, current_obstacles);
    online_data.block(ACADO_N, 0, 1, ACADO_NOD) << tmp.transpose().matrix();

    Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1 >>(const_cast<double *>(acadoVariables.x0)) = x_0;
    Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N >>(const_cast<double *>(acadoVariables.y)) =
            reference.transpose();
    Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1 >>(const_cast<double *>(acadoVariables.yN)) =
            referenceN.transpose();
    Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N + 1 >>(const_cast<double *>(acadoVariables.od)) =
            online_data.transpose();

    ros::WallTime time_before_solving = ros::WallTime::now();
    int acado_status;
    for (int iter = 0; iter < 5; iter++) {/* Prepare for the RTI step. */
        acado_preparationStep();

        /* Compute the feedback step. */
        acado_status = acado_feedbackStep();
        if (acado_getKKT() < 0.01) {
            break;
        }
    }

    solve_time_avg += (ros::WallTime::now() - time_before_solving).toSec() * 1000.0;

    double vel_x_ref = acadoVariables.u[0];
    double vel_y_ref = acadoVariables.u[1];
    double vel_yaw_robot_ref = acadoVariables.u[2];
    double vel_yaw_cam_ref = acadoVariables.u[3];

    if (std::isnan(vel_x_ref) || std::isnan(vel_y_ref) || std::isnan(vel_yaw_cam_ref)
        || std::isnan(vel_yaw_robot_ref) || acado_status != 0) {
        cnt_failed += 1;
        ROS_WARN_STREAM("Nonlinear MPC: Solver failed with status: " << acado_status);
        ROS_WARN_STREAM("Error " << acado_getErrorString(acado_status));
        ROS_WARN("reinitializing...");
        initializeSolver(x_0);
        pubRobotCmd.publish(cmd);
        pubCamCmd.publish(cmd);
        return;
    }

    cnt_failed = 0;

    cmd.angular.z = vel_yaw_cam_ref;
    pubCamCmd.publish(cmd);
    cmd.linear.x = vel_x_ref;
    cmd.linear.y = vel_y_ref;
    cmd.angular.z = vel_yaw_robot_ref;
    pubRobotCmd.publish(cmd);

    last_valid_plan_ = ros::Time::now();

    state = Eigen::Map<Eigen::Matrix<double, ACADO_N + 1, ACADO_NX, Eigen::RowMajor >>(acadoVariables.x);

    double diff_time = (ros::WallTime::now() - start_time).toSec();

    if (verbose) {
        static int counter = 0;
        if (counter > 100) {
            ROS_INFO_STREAM("average solve time: " << solve_time_avg / counter << " ms");
            solve_time_avg = 0.0;

            ROS_INFO_STREAM("Controller loop time : " << diff_time * 1000.0 << " ms");

            ROS_INFO_STREAM(
                    "vel x: " << vel_x_ref << "\t" << "vel y: \t" << vel_y_ref << "\t" << "yaw robot ref : \t"
                              << vel_yaw_robot_ref << "\t" << "yaw cam ref : \t" << vel_yaw_cam_ref << "\t");
            counter = 0;
        }
        counter++;
    }
}

bool compareObs(Eigen::Vector3d o1, Eigen::Vector3d o2) {
    return o1(2) < o2(2);
}

Eigen::VectorXd nlmpc::computeObsPos(const int j, const double r_x, const double r_y, double covariance_radius,
                                     const costmap_converter::ObstacleArrayMsg &obstacleArrayMsg) {
    selfPoseTraj.poses.clear();
    geometry_msgs::Pose tmpPose;

    Eigen::VectorXd finalObsPos(ACADO_NOD);
    Position3D virtualPoint(r_x, r_y, 0);
    if (j != 0) {
        double x = state.block(j, 0, 1, 2).coeff(0, 0);
        double y = state.block(j, 0, 1, 2).coeff(0, 1);
        virtualPoint(0) = x;
        virtualPoint(1) = y;
    }
    virtualPoint(2) = 0; // zero height
    tmpPose.position.x = virtualPoint(0);
    tmpPose.position.y = virtualPoint(1);
    tmpPose.position.z = 0;
    selfPoseTraj.poses.push_back(tmpPose);

    std::vector<Eigen::Vector3d> wantedObs;
    std::vector<Eigen::Vector3d> otherObs;

    try {
        listener_.lookupTransform("/map", "odom", ros::Time::now(), map_to_odom);
    }
    catch (tf2::TransformException &e) {
        ROS_WARN_STREAM("using old tf");
    }

    for (int i = 0; i < obstacleArrayMsg.obstacles.size(); i++) {
        double x_obs;
        double y_obs;
        double z_obs = 0;
        double dist = 10000;  //the max distance within the local costmap is just 6m, this is a safety value
        double x_near, y_near;

        for (auto point : obstacleArrayMsg.obstacles.at(i).polygon.points) {

            tf::Transform odom_to_target;
            odom_to_target.setOrigin(tf::Vector3(point.x, point.y, 0));
            odom_to_target = map_to_odom * odom_to_target;

            x_obs = odom_to_target.getOrigin().x();
            y_obs = odom_to_target.getOrigin().y();

            x_obs += pred_sampling * obstacleArrayMsg.obstacles.at(i).velocities.twist.linear.x * j;
            y_obs += pred_sampling * obstacleArrayMsg.obstacles.at(i).velocities.twist.linear.y * j;

            double theta = std::atan2((virtualPoint(1) - y_obs), (virtualPoint(0) - x_obs));
            x_obs += cos(theta) * covariance_radius;
            y_obs += sin(theta) * covariance_radius;
            Position3D obsPosition(x_obs, y_obs, z_obs);
            Position3D posDiff = virtualPoint - obsPosition;
            posDiff(2) = 0; //@HACK : 2-D distance only

            if (dist > posDiff.norm() && !isnan(posDiff.norm())) {
                x_near = x_obs;
                y_near = y_obs;
                dist = posDiff.norm();
            }
        }

        // dynamic obstacle
        if (obstacleArrayMsg.obstacles.at(i).velocities.twist.linear.x != 0 ||
            obstacleArrayMsg.obstacles.at(i).velocities.twist.linear.y != 0) {
            if (dist > 0.25 && !isnan(dist))
                wantedObs.push_back(Eigen::Vector3d(x_near, y_near, dist));
        } else {
            if (dist > 0.25 && !isnan(dist))
                otherObs.push_back(Eigen::Vector3d(x_near, y_near, dist));
        }
    }
    if (wantedObs.size() < ACADO_NOD / 2) {
        double cnt = 0;
        if (otherObs.size())
            std::sort(otherObs.begin(), otherObs.end(), compareObs);

        while (wantedObs.size() < ACADO_NOD / 2 && cnt < otherObs.size()) {
            wantedObs.push_back(otherObs.at(cnt));
            cnt += 1;
        }
        while (wantedObs.size() < ACADO_NOD / 2) {
            wantedObs.push_back(Eigen::Vector3d(90, 90, 90));
        }
    }

    std::sort(wantedObs.begin(), wantedObs.end(), compareObs);

    int cnt = 0;
    for (auto tmp : wantedObs) {
        finalObsPos(cnt) = tmp.coeff(0);
        finalObsPos(cnt + ACADO_NOD / 2) = tmp(1);
        cnt += 1;
//        if (verbose)
//            ROS_INFO_STREAM(
//                    "obs " << cnt << " in pos x: " << tmp(0) << " y: " << tmp(1) << " dist " << tmp(2) << " value "
//                           << exp(1 - 5 * tmp(2)));
        if (cnt == 10)
            break;
    }

    selfPoseTraj.header.frame_id = "map";
    selfPoseTraj.header.stamp = ros::Time::now();
    pubPoseTraj.publish(selfPoseTraj);

    return finalObsPos;
}

void nlmpc::initializeSolver(Eigen::VectorXd x_0, bool first) {
    applyParam();
    for (int i = 0; i < ACADO_N + 1; i++) {
        state.block(i, 0, 1, ACADO_NX) << x_0.transpose();
    }
    // set current state as reference, only if it's the first re-init
    if (first) {
        referenceN << x_0(0), x_0(1), x_0(3);
        for (size_t i = 0; i < ACADO_N; i++) {
            reference.block(i, 0, 1, ACADO_NY)
                    << 0, 0, 0, 0, x_0(0), x_0(1), x_0(3), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            // vel_{x,y,y_rob,y_cam}, final_orientation, obstacles potential{1-10}
        }
    }

    Eigen::Map<Eigen::Matrix<double, ACADO_NX, ACADO_N + 1 >>(const_cast<double *>(acadoVariables.x)) =
            state.transpose();
    Eigen::Map<Eigen::Matrix<double, ACADO_NU, ACADO_N >>(const_cast<double *>(acadoVariables.u)) =
            input.transpose();
    Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N >>(const_cast<double *>(acadoVariables.y)) =
            reference.transpose();
    Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1 >>(const_cast<double *>(acadoVariables.yN)) =
            referenceN.transpose();
}


void nlmpc::ControllerDynConfigCallBack(robotino_mpc::NonLinearMPCConfig &config, uint32_t level) {
    Eigen::Vector4d pen_vel;
    Eigen::Vector3d pen_pos;
    Eigen::VectorXd pen_obs(10);

    Eigen::VectorXd control_limits(7);
    //double v_x_limit, v_y_limit, v_yaw_cam_limit, v_yaw_rob_limit;

    pen_vel << config.q_vel_x, config.q_vel_y, config.q_vel_yaw_robot, config.q_vel_yaw_cam;
    pen_obs.setConstant(config.q_obs);
    pen_pos << config.q_x, config.q_y, config.q_yaw;

    control_limits << config.v_x, config.v_y, config.v_tr,
            config.v_yaw_rob, config.v_yaw_cam, config.v_rot,
            config.min_dist;

    prev_min_dist = -1;

    setPosPen(pen_pos);
    setVelPen(pen_vel);
    setObsPen(pen_obs);
    setLimits(control_limits);

    dist_tolerance = config.dist_tolerance;
    angle_tolerance = config.angle_tolerance;

    oscillation_timeout_ = config.oscillation_timeout_;

    max_planning_retries_ = config.max_planning_retries_;
    oscillation_reset_dist_ = config.oscillation_reset_dist_;
    oscillation_reset_angle_ = config.oscillation_reset_angle_;
    planner_patience_ = config.planner_patience_;

    applyParam();
}

