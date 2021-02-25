#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>


int main(int argc, char **argv)
{
    //
    nav_msgs::OccupancyGrid occupancy_grid_msg;

    // Init OccupancyGrid msg
    occupancy_grid_msg.header.frame_id = "map";
    occupancy_grid_msg.info.resolution = 0.05;
    occupancy_grid_msg.info.width = 621;
    occupancy_grid_msg.info.height = 621;

    geometry_msgs::Point pose_position;
    pose_position.x = -15.525;
    pose_position.y = -15.525;
    pose_position.z = 0;

    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
    geometry_msgs::Quaternion pose_orientation;
    quaternionTFToMsg(q, pose_orientation);

    geometry_msgs::Pose origin_pose;
    origin_pose.position = pose_position;
    origin_pose.orientation = pose_orientation;

    occupancy_grid_msg.info.origin = origin_pose;

    // Get path and file name
    std::string package_path = ros::package::getPath("robotino_simulations");
    std::string test_results_path = "/home/ebonetto/Desktop/Test/E1/occupancy.txt";
    std::string ground_truth_map_path ="/home/ebonetto/Desktop/Test/E1/gt_occupancy.txt";

    std::ifstream test_map_file(test_results_path.c_str());
    std::ifstream ground_truth_map_file(ground_truth_map_path.c_str());

    std::string line;
    int i = 0;


    if (ground_truth_map_file.is_open()){
        if (test_map_file.is_open()){
            while (std::getline(test_map_file, line)){
                std::stringstream ss_test;
                ss_test << line;
                double p_test;
                ss_test >> p_test;

                std::getline(ground_truth_map_file, line);
                std::stringstream ss_ref;
                ss_ref << line;
                double p_ref;
                ss_ref >> p_ref;

                // Map Scoring
                if (p_test == -1){
                    p_test = 50;
                }
                if (p_ref == -1){
                    p_ref = 50;
                }
                occupancy_grid_msg.data.push_back(std::abs(p_test - p_ref));
            }
        }
        else {
            ROS_INFO("Failed to open test_map_file!");
        }
    }
    else {
        ROS_INFO("Failed to open ground_truth_map_file!");
    }

    ros::init(argc, argv, "diff_map_pub");
    ros::NodeHandle nh;
    ros::Publisher map_pub;
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("diff_map", 1);

    while (ros::ok()){
        map_pub.publish(occupancy_grid_msg);
        ros::spinOnce();
    }

    return 0;
}