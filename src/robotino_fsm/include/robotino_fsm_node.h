#ifndef RTABMAP_ROS_ROBOTINO_FSM_NODE_H
#define RTABMAP_ROS_ROBOTINO_FSM_NODE_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
#include "move_base/move_base.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "dynamic_reconfigure/Reconfigure.h"
#include "rtabmap_ros/Info.h"
#include "rtabmap_ros/GetNearbyNodeData.h"
#include "rtabmap/core/Link.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include "tf/transform_listener.h"

#include "active_slam/get_frontier_list.h"
#include "active_slam/get_best_path.h"
#include "active_slam/get_best_head.h"

#include "robot_localization/SetPose.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

class robotino_fsm_node {

public:
    enum {
        OPERATING, RECEIVED, REACHED, OSCILLATING, INVALID, WAITING
    };

    robotino_fsm_node(ros::NodeHandle nh, ros::NodeHandle private_nh);

    ~robotino_fsm_node();

private:
    void init_vars();

    ros::NodeHandle nh_, private_nh_;
    ros::AsyncSpinner spinner;

    // robot and camera positions + orientation
    bool tf_ok, require_new_goal, replan, send_goal, done_360, filter_last, only_last_set, mid_optimizer, pre_fix,sim;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MyApproxSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, nav_msgs::Odometry> MyExactSyncPolicy;
    typedef message_filters::Synchronizer<MyApproxSyncPolicy>  sync_approx;
    typedef message_filters::Synchronizer<MyExactSyncPolicy>  sync_exact;
    boost::shared_ptr<sync_approx> approx;
    boost::shared_ptr<sync_exact> exact;

    double cov_pos, yaw_rob, yaw_cam; // determinant of the covariance relative just to the x, y positions
    int max_fail_before_new_goal, failed, failed_plans, failed_replans, kind_cost, replan_cnt;
    int is_second;

    geometry_msgs::Pose globalRobot, globalCam;
    std::string odom_robot_topic, odom_cam_topic;

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odomRobot, const nav_msgs::Odometry::ConstPtr &odomCam);
		tf2_ros::Buffer tfBuffer_;
		tf2_ros::TransformListener tfListener_;
		tf::TransformListener listener_;

    void transform_odom(const nav_msgs::Odometry::ConstPtr &odomRobot, const nav_msgs::Odometry::ConstPtr &odomCam);
    void transform_pose(geometry_msgs::Pose &pose);

    void new_goal();

    void general_reset();

    // get path toward goal
    // ros::Publisher pathReq; + listener bah
    ros::ServiceClient pathReq_client, get_frontier, get_best_path, get_best_head, clear_costmap, clear_obstacles;
    ros::ServiceClient rtabmap_pause, rtabmap_resetodom, rtabmap_newmap, rtabmap_resume;
    ros::ServiceClient ekf_camera_reset, ekf_robot_reset;

    std::string get_plan_topic, get_nearby_nodes_topic, get_frontier_topic, get_best_path_topic, get_best_head_topic;

    std::vector<geometry_msgs::Pose> getFrontierPoints();

    ros::ServiceClient nearby_nodes_client;

    std::vector<geometry_msgs::PoseStamped>
    getPlan(const geometry_msgs::Pose end, const geometry_msgs::PosePtr start = nullptr);

    std::vector<geometry_msgs::Pose>
    getBestPath(const std::vector<std::pair<std::vector<geometry_msgs::Pose>, double>> &paths, int kind = 1,
                int fov = 86);

    std::vector<geometry_msgs::Pose>
    refinePlan(std::vector<geometry_msgs::PoseStamped> plan, double &tot_dist, double thr = 0.5);

    bool find_lc();

    int counter_lc;
    bool found_lc;

    bool emergency;

    std::vector<geometry_msgs::PoseStamped> revisit_goal();
    std::vector<geometry_msgs::Pose> currentPlan;

    void replan_goal();

    // local planning
    std::string mpcStateTopic, mpcGoalTopic;
    ros::Subscriber mpcStateClient;
    ros::Publisher mpcGoalPub, pubCamCmd,pubRobCmd;

    void mpcStateCallback(const std_msgs::Int8::ConstPtr &msg);

    void rtabmapInfoCallback(const rtabmap_ros::Info::ConstPtr &msg);

    std::vector<int> loop_clos;

    ros::ServiceClient opt_feat_head;
    ros::ServiceClient waypoint_map_info;
    ros::Subscriber mpc_path;
    ros::Subscriber rtabmap_info;
    ros::Publisher mpc_goal;
};


#endif //RTABMAP_ROS_ROBOTINO_FSM_NODE_H
