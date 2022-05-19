#include <chrono>
#include "../include/camera_best_heading.h"

using namespace std::chrono;

enum LoS {
    Free = 0, Unknown = 1, Occupied = 10
};

bool
camera_best_heading::getLoS(const octomap::OcTree &octree, const cv::Point3f &start, const cv::Point3f &end,
                            double thr_dist) {

    octomap::point3d end2;

    double dist = std::pow(
            (end.x - start.x) * (end.x - start.x) +
            (end.y - start.y) * (end.y - start.y) +
            (end.z - start.z) * (end.z - start.z), 0.5);

    bool hit = octree.castRay(octomap::point3d(start.x, start.y, start.z),
                              octomap::point3d((start.x - end.x) / dist, (start.y - end.y) / dist,
                                               (start.z - end.z) / dist), end2, true, thr_dist);
    // no obs found or maxRange reached
    if (!hit) {
        return true;
    }
    // if obs found ged distance. If dist obs >= dist feature then feat is visible.
    double dist2 = end2.distance(octomap::point3d(start.x, start.y, start.z));
    if (dist2 >= dist) {
        return true;
    }
    return false;
}

void camera_best_heading::filter_features(cv::Point3f mpc_pose, std::vector<cv::Point3f> &kp3D,
                                          std::vector<cv::KeyPoint> &kpInfo,
                                          std::vector<double> &margins) {
    double min_angle = 2 * M_PI;
    double max_angle = -2 * M_PI;

    geometry_msgs::Pose p;

    auto it_info = kpInfo.begin();

    for (auto it_3D = kp3D.begin(); it_3D != kp3D.end();) {
        if (std::isnan(it_3D->x) || std::isnan(it_3D->y) ||
            std::isnan(it_3D->z)) {
            it_info = kpInfo.erase(it_info);
            it_3D = kp3D.erase(it_3D);
        } else {
            p.position.x = it_3D->x;
            p.position.y = it_3D->y;
            p.position.z = it_3D->z;
            tf2::doTransform(p, p, transformStamped);
            it_3D->x = p.position.x;
            it_3D->y = p.position.y;
            it_3D->z = p.position.z;


            double dist = std::pow(
                    (it_3D->x - mpc_pose.x) * (it_3D->x - mpc_pose.x) +
                    (it_3D->y - mpc_pose.y) * (it_3D->y - mpc_pose.y) +
                    (it_3D->z - mpc_pose.z) * (it_3D->z - mpc_pose.z), 0.5);


            if (dist > max_feat_dist ||
                !getLoS(*ocTree_, mpc_pose, cv::Point3f(it_3D->x, it_3D->y, it_3D->z),
                        max_feat_dist)) {
                it_info = kpInfo.erase(it_info);
                it_3D = kp3D.erase(it_3D);
            } else {
                double angle = std::atan2(it_3D->y - mpc_pose.y, it_3D->x - mpc_pose.x);
                angle < 0 ? angle += 2 * M_PI : angle;

                if (angle < min_angle) {
                    margins.at(0) = angle;
                    min_angle = angle;
                }
                if (angle > max_angle) {
                    margins.at(1) = angle;
                    max_angle = angle;
                }
                ++it_info;
                ++it_3D;
            }
        }
    }
}

std_msgs::Float32MultiArray camera_best_heading::getPrediction() {
    if (last_id == -1 || tree == nullptr || tree->size() == 0)
        return std_msgs::Float32MultiArray{};


    Eigen::Quaternion<double> quat = mapToOdom.getQuaterniond() * signatures.at(last_id).getPose().getQuaterniond();
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    transformStamped.transform.translation.x =
            mapToOdom.translation().x() + signatures.at(last_id).getPose().translation().x();
    transformStamped.transform.translation.y =
            mapToOdom.translation().y() + signatures.at(last_id).getPose().translation().y();
    transformStamped.transform.translation.z =
            mapToOdom.translation().z() + signatures.at(last_id).getPose().translation().z();

    // 2 angles for the 2D margins
    std::vector<double> margins;
    margins.resize(2);

    // consider only the features that are near enough, in the FOV and not occluded
    std::vector<cv::Point3f> kp3D = signatures.at(last_id).getWords3();
    if (kp3D.empty()) {
        ROS_WARN("kp3D is empty");
        return std_msgs::Float32MultiArray{};
    }
    std::vector<cv::KeyPoint> kpInfo = signatures.at(last_id).getWordsKpts();

    filter_features(targetPose, kp3D, kpInfo, margins);

    // current_feat now it's filtered
    if (!kp3D.empty()) {
        double delta = std::fmod((margins.at(0) - margins.at(1) + M_PI + 2 * M_PI), (2 * M_PI)) - M_PI;
        std::vector<int> angles = find_best_pov(targetPose, kp3D, kpInfo, margins, delta);
        std_msgs::Float32MultiArray theta;
        for (auto i : angles)
            theta.data.push_back(std::fmod(i + fov / 2, 360));
        return theta;
    } else {
        return std_msgs::Float32MultiArray{}; // I don't have features -> no answer from the service
    }
}


std::vector<int> camera_best_heading::find_best_pov(const cv::Point3f &mpc_pose, std::vector<cv::Point3f> &kp3D,
                                                    std::vector<cv::KeyPoint> &kpInfo,
                                                    std::vector<double> &margins, double delta) {

    std::vector<std::pair<int, float>> features;

    double center_angle = std::fmod((margins.at(1) + delta / 2), 2 * M_PI);
    bool zero_inside = false;
    if ((fmod((center_angle - 0 + M_PI + 2 * M_PI), (2 * M_PI)) - M_PI) <= abs(delta / 2) &&
        (fmod((center_angle - 0 + M_PI + 2 * M_PI), (2 * M_PI)) - M_PI) >= -abs(delta / 2)) {
        zero_inside = true;
    }

    auto it_info = kpInfo.begin();
    int min_angle = 0;
    for (auto it_3D = kp3D.begin(); it_3D != kp3D.end();) {

        int angle = 0;
        if (!std::isnan(std::atan2(it_3D->y - mpc_pose.y, it_3D->x - mpc_pose.x))) {
            angle = (int) std::round(
                    std::atan2(it_3D->y - mpc_pose.y, it_3D->x - mpc_pose.x) * (180 / M_PI));
            min_angle = std::min(angle, min_angle);
        } else {
            ++it_info;
            ++it_3D;
            continue;
        }
        angle < 0 ? angle += 360 : angle;
        if (zero_inside) {
            if (angle < fov) {
                features.emplace_back(std::pair<int, float>(angle + 360, it_info->response));
            }
        }
        features.emplace_back(std::pair<int, float>(angle, it_info->response));
        ++it_info;
        ++it_3D;
    }

    std::sort(features.begin(), features.end());

    std::vector<std::pair<int, float>> max_angle_res;
    // span of features can be < fov
    int max_angle = std::max(features.at(features.size() - 1).first, features.at(0).first + fov);
    if (zero_inside)
        max_angle = std::max(features.at(features.size() - 1).first, min_angle + 360 + fov);

    for (auto i: features) {
        bool found = false;
        for (auto &j : max_angle_res) {
            if (j.first > i.first - fov) {
                if (feat_based)
                    j.second = j.second + i.second;
                else
                    j.second = j.second + 1;
            }
            if (j.first == i.first) {
                found = true;
            }
        }

        if (!found && i.first <= max_angle - fov + 1) {
            if (feat_based) {
                max_angle_res.emplace_back(std::pair<int, float>(i.first, i.second));
            } else {
                max_angle_res.emplace_back(std::pair<int, float>(i.first, 1));
            }
        }
    }

    float max_utility = -1;
    std::vector<int> angles;
    for (auto i: max_angle_res) {
        if (max_utility < i.second) {
            max_utility = i.second;
            angles.clear();
            angles.emplace_back(i.first);
        } else if (max_utility == i.second) {
            angles.emplace_back(i.first);
        }
    }
    return angles;
}

void camera_best_heading::getOctomapCallback(octomap_msgs::Octomap msg) {
    tree = octomap_msgs::msgToMap(msg);
    if (tree->size() > 0) {
        delete ocTree_;
        ocTree_ = nullptr;
        ocTree_ = static_cast<octomap::OcTree *>(tree); // dynamic_cast not working
    }
}

void camera_best_heading::getMapDataCallback(rtabmap_ros::MapData msg) {
    signatures.clear();
    links.clear();
    poses.clear();
    last_id = msg.graph.posesId.at(msg.graph.posesId.size() - 1);
    rtabmap_ros::mapDataFromROS(msg, poses, links, signatures, mapToOdom);
}

camera_best_heading::~camera_best_heading() {}

camera_best_heading::camera_best_heading(ros::NodeHandle
                                         nh, ros::NodeHandle private_nh) : nh_(nh), private_nh_(private_nh) {
    ocTree_ = nullptr;
    last_id = -1;

    private_nh_.param<int>("fov", fov, 85);
    private_nh_.param<double>("max_feat_dist", max_feat_dist, 10);
    private_nh_.param<bool>("feat_based", feat_based, true);
    private_nh_.param<double>("camera_height", camera_height, 0.85);

    get_octomap_sub = nh_.subscribe<octomap_msgs::Octomap>("/rtabmap/octomap_full",
                                                           100, &camera_best_heading::getOctomapCallback, this);
    get_mapdata_sub = nh_.subscribe<rtabmap_ros::MapData>("/rtabmap/mapData",
                                                          100, &camera_best_heading::getMapDataCallback, this);
    subDest = nh_.subscribe<nav_msgs::Odometry>("/waypoint", 1, boost::bind(&camera_best_heading::getTarget, this, _1));

    heading = nh_.advertise<std_msgs::Float32MultiArray>("/opt_head", 10);
}

void camera_best_heading::getTarget(nav_msgs::Odometry::ConstPtr targetMsg) {
    if (targetMsg != nullptr)
        targetPose = cv::Point3f(targetMsg->pose.pose.position.x, targetMsg->pose.pose.position.y, camera_height);
    else
        targetPose = cv::Point3f(0, 0, 0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_best_heading");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    camera_best_heading cameraPrediction(nh, private_nh);
    while (ros::ok){
        std_msgs::Float32MultiArray msg_out = cameraPrediction.getPrediction();
        cameraPrediction.heading.publish(msg_out);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}