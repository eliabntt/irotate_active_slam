/**
 *  map_extract.h
 */

#ifndef MAP_EXTRACT_H
#define MAP_EXTRACT_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>

#include <string>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include "active_slam/get_best_path.h"
#include "active_slam/get_best_head.h"
#include <nav_msgs/GetMap.h>
#include <queue>
#include <boost/filesystem.hpp>

#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <cstdint>
#include <unistd.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <stdlib.h>

#undef min
#undef max

using std::max;
using std::min;


class MapExtract {
public:
    /**
     *  Class Constructor
     */
    MapExtract(ros::NodeHandle nh, ros::NodeHandle private_nh);

    /**
     *  Class Destructor
     */
    ~MapExtract();

    /**
     *  Helper function which initialises some parameters
     */
    void initParams();

    /**
     *  Get the best path with headings among set of possibilities
     */
    bool getBestPathServiceCallback(
            active_slam::get_best_path::Request &request,
            active_slam::get_best_path::Response &response);

    /**
     *  This service callback provides information about the singular POV.
     */
    bool getBestHeadServiceCallback(
            active_slam::get_best_head::Request &request,
            active_slam::get_best_head::Response &response);

private:
    void prepare_messages(double map_resolution_);

    void publish_bb(double x_max, double x_min, double y_max, double y_min, double x_origin, double y_origin,
                    double map_resolution_);

    void getBoundingBox(const double &theta, const double &fov, const double &x_start, const double &y_start,
                        double &x_max, double &x_min, double &y_max, double &y_min);

    void cellInFov(double const &theta_start, double const &fov, double const &x_start, double const &y_start,
                   std::vector<std::pair<double, double>> &occuVector, // angle, distance
                   std::vector<std::pair<double, double>> &min_max_angle_occu_vec, // min and max angles for each occupied cell
                   std::vector<std::pair<double, double>> &cellPos, // x,y
                   std::vector<std::pair<double, double>> &min_max_angle_vec,
                   std::vector<std::pair<double, double>> &cellVector,
                   std::vector<int> &prob_vec, const std::vector<bool> & added_before);

    void raytracing(std::vector<std::pair<double, double>> &occuVector, // angle, distance
                    std::vector<std::pair<double, double>> &min_max_angle_occu_vec, // min and max angles for each occupied cell
                    const std::vector<std::pair<double, double>> &cellPos, // x,y
                    const std::vector<std::pair<double, double>> &min_max_angle_vec,
                    std::vector<std::pair<double, double>> &cellVector,
                    const std::vector<int> &prob_vec,
                    std::vector<std::pair<int, int>> &visible_cells,
                    std::vector<std::pair<double, double>> &visibleCellsPos,  const std::vector<bool> & added_before);

    double getMaxUtility(std::vector<std::pair<int, int>> &visible_cells,
                         const std::vector<std::pair<double, double>> &visibleCellsPos,
                         const double &x_start, const double &y_start, std::vector<int> &max_utility_angles,
                         const double &wp_dist, const double &path_length, const double &inter_angle);

    double cost_function(const int &kind_cost, const int &prob, const double &dist, const double &length);

    bool getMap();

    double normalizeToResolution(const double &coord);

    void clear_messages();

    ros::Publisher marker_pub;

    // Rosparam         ;
    double radius_camera_, min_distance_, fov_camera_rad;
    int prob_threshold_free_, kind_cost_;
    bool is_zero_inside,debug_, weighted_avg;
    int fov_camera_deg_;

    float map_resolution_, x_origin, y_origin;
    int map_width_, map_height_;

    // Node handler
    ros::NodeHandle nh_, private_nh_;

    // Ros msgs
    geometry_msgs::Pose origin_pose;
    nav_msgs::OccupancyGrid latest_map_msg_;

    // Service
    ros::ServiceClient get_map_prob_client_;
    ros::ServiceServer set_path_service_, set_point_service_;

    // Publisher
    ros::Publisher occ_cell_pub_, free_cell_pub_, unk_cell_pub_;

    ros::Publisher raytracing_target_pub_, raytracing_occu_pub_, raytracing_free_pub_, raytracing_unk_pub_;

    ros::Publisher origin_fov_pub_, submap_bounding_box_cell_pub_;

    ros::Publisher opt_heading_pub_;

    nav_msgs::GridCells occ_points_msg, free_points_msg, unk_points_msg, raytracing_target_msg, raytracing_free_msg, raytracing_occu_msg, opt_heading_msg, raytracing_unk_msg;


    ros::Time start_time_;
};

#endif  // MAP_EXTRACT_H
