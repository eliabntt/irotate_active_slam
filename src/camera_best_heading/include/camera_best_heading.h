#ifndef CAMERA_BEST_HEADING_H
#define CAMERA_BEST_HEADING_H

#include <string>
#include <stdlib.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/GetOctomap.h"
#include "octomap/octomap.h"

#include "rtabmap_ros/GetNodeData.h"
#include "rtabmap_ros/MsgConversion.h"
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/OctoMap.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "rtabmap/core/OctoMap.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32MultiArray.h>

#include <nav_msgs/Odometry.h>

#define M_PI 3.14159265358979323846

class camera_best_heading {
public:
    camera_best_heading(ros::NodeHandle nh, ros::NodeHandle private_nh);

    ~camera_best_heading();

    std_msgs::Float32MultiArray getPrediction();
    ros::Publisher heading;

private:
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber get_octomap_sub, get_mapdata_sub, subDest;

    void getTarget(nav_msgs::Odometry::ConstPtr targetMsg);

    cv::Point3f targetPose;

    octomap::AbstractOcTree *tree;

    void getMapDataCallback(rtabmap_ros::MapData);

    void getOctomapCallback(octomap_msgs::Octomap msg);

    octomap::OcTree *ocTree_;

    geometry_msgs::TransformStamped transformStamped;
    int last_id;

    int fov;
    double max_feat_dist, camera_height;
    bool feat_based;

    octomap::KeyRay keyRay;
    octomap::OcTreeKey end_key;

    rtabmap::Transform mapToOdom;
    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> links;
    std::map<int, rtabmap::Signature> signatures;

    bool getLoS(const octomap::OcTree &octree, const cv::Point3f &start, const cv::Point3f &end,
                double threshold_dist = 3);

    void filter_features(cv::Point3f mpc_pose, std::vector<cv::Point3f> &kp3D,
                                          std::vector<cv::KeyPoint> &kpInfo,
                                          std::vector<double> &margins);
    
    std::vector<int> find_best_pov(const cv::Point3f &mpc_pose, std::vector<cv::Point3f> &kp3D,
                                                    std::vector<cv::KeyPoint> &kpInfo,
                                                    std::vector<double> &margins, double delta);

};


#endif //CAMERA_BEST_HEADING_H
