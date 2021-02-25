#ifndef FRONTIER_EXPLORATION_H
#define FRONTIER_EXPLORATION_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <queue>
#include <geometry_msgs/Pose2D.h>
#include <active_slam/get_frontier_list.h>
#include <nav_msgs/GetMap.h>
#include <unistd.h>
#include <math.h>
#include <cmath>


class FrontierExploration {
public:
    /**
     *  Class Constructor
     */
    FrontierExploration(ros::NodeHandle nh);

    /**
     *  Class Destructor
     */
    ~FrontierExploration();

    /**
     *  This service callback calculates the frontier centroid informations in
     *  the actual map, returns the centroid information and publishes a cell grid
     *  with the centroids.
     */

    bool getFrontierServiceCallback(
            active_slam::get_frontier_list::Request &request,
            active_slam::get_frontier_list::Response &response);


    /**
     *  Takes the inital cell id and a frontier flag to search for the frontier
     *  region around the initial cell. Returns a Pose2D centroid information.
     *  If the frontier size is too small it returns all values as zero.
     */

    bool getFrontierCentroid(unsigned int initial_cell,
                             std::vector<bool> &frontier_flag,
                             int map_width,
                             int map_height,
                             float resolution,
                             geometry_msgs::Pose2D &centroid);

    /**
     *  Transforms a cell id to x, y world coordinates.
     */
    void idToWorldXY(unsigned int id, double &x, double &y, double &theta, int map_width,
                     int map_height, float map_resolution, double x_start, double y_start, double x_offset, double y_offset);

    /**
     *  Returns a list of cell id's for the neighbourhood around #x cells.
     */
    std::vector<unsigned int> neighbourXCells(unsigned int id, int map_width,
                                              int map_height, unsigned int n_cells);

    /**
     *  Returns a list of cell id's for the 8-neighbourhood around an id cell.
     */
    std::vector<unsigned int> neighbour8(unsigned int id, int map_width,
                                         int map_height);

    /**
     *  Returns a list of cell id's for the 4-neighbourhood around an id cell.
     */
    std::vector<unsigned int> neighbour4(unsigned int id, int map_width,
                                         int map_height);


private:
    // Rosparam
    int frontier_size_;

    // Node handler
    ros::NodeHandle nh_;

    ros::Publisher frontier_cell_pub_;

    // Service
    ros::ServiceServer get_frontier_service_;
    ros::ServiceClient get_map_prob_client_;

    // Ros msgs and srv
    nav_msgs::OccupancyGrid latest_map_msg_;

    // Tf
    tf::TransformListener robot_pose_listener_;

    // map
    std::string header;
    int map_width_;
    int map_height_;
    float map_resolution_;
    double x_offset, y_offset;

    double x_robot, y_robot;
    double theta_robot;

    // additional
    double x_diff;
    double y_diff;

};

#endif // FRONTIER_EXPLORATION_H
