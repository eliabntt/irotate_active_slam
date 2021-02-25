#include <frontier_exploration.h>

/**
 * TODO: frontier exploration
 *
 * occupied: 100
 * free: 0
 * unknown: -1
 *  
 * Source: https://github.com/ethz-asl/crowdbot_active_slam
 *
 */

/**
 *  function which creates map index from position information.
 */
std::vector<int> positionToMapIndex(double x, double y,
                                    double offset_x, double offset_y, float map_resolution) {
    std::vector<int> index(2);
    index[0] = floor((x - offset_x) / map_resolution);
    index[1] = floor((y - offset_y) / map_resolution);

    return index;
}

/**
* function which calculates the angle from x and y.
*/
double xyDiffToYaw(double x_diff, double y_diff) {
    if (x_diff == 0 && y_diff > 0) {
        return M_PI_2;
    } else if (x_diff == 0 && y_diff < 0) {
        return -M_PI_2;
    } else if (x_diff < 0 && y_diff == 0) {
        return M_PI;
    } else if (x_diff < 0 && y_diff < 0) {
        return atan(y_diff / x_diff) - M_PI;
    } else if (x_diff < 0 && y_diff > 0) {
        return atan(y_diff / x_diff) + M_PI;
    } else {
        return atan(y_diff / x_diff);
    }
}

FrontierExploration::FrontierExploration(ros::NodeHandle nh)
        : nh_(nh) {
    // Get ros param
    nh_.param<int>("frontier_exploration/frontier_size", frontier_size_, 50);

    // Init publisher
    frontier_cell_pub_ = nh_.advertise<nav_msgs::GridCells>("frontier_points_grid_cell", 1);

    // Init service
    get_frontier_service_ = nh_.advertiseService("frontier_exploration_service",
                                                 &FrontierExploration::getFrontierServiceCallback, this);
    ROS_INFO("frontier_exploration_service started!");

    // Init service clients
    get_map_prob_client_ = nh_.serviceClient<nav_msgs::GetMap>
            ("/rtabmap/get_map");
}

FrontierExploration::~FrontierExploration() {}


bool FrontierExploration::getFrontierServiceCallback(
        active_slam::get_frontier_list::Request &request,
        active_slam::get_frontier_list::Response &response) {
    // Frontier exploration calculations
    // based on implementation http://wiki.ros.org/frontier_exploration

    ROS_INFO("frontier_exploration_service continue!");

    // Init Transform listener and get current robot pose
    tf::StampedTransform robot_pose_tf;
    robot_pose_listener_.waitForTransform("map", "base_link", ros::Time(0),
                                          ros::Duration(1.0));
    robot_pose_listener_.lookupTransform("map", "base_link",
                                         ros::Time(0), robot_pose_tf);

    x_robot = robot_pose_tf.getOrigin().getX();
    y_robot = robot_pose_tf.getOrigin().getY();

    if (fmod(x_robot, map_resolution_) != 0) {
        if (fmod(x_robot, map_resolution_) > map_resolution_ / 2) {
            x_robot = x_robot + (map_resolution_ - fmod(x_robot, map_resolution_));
        } else {
            x_robot = x_robot - fmod(x_robot, map_resolution_);
        }
    }
    if (fmod(y_robot, map_resolution_) != 0) {
        if (fmod(y_robot, map_resolution_) > map_resolution_ / 2) {
            y_robot = y_robot + (map_resolution_ - fmod(y_robot, map_resolution_));
        } else {
            y_robot = y_robot - fmod(y_robot, map_resolution_);
        }
    }

    theta_robot = tf::getYaw(robot_pose_tf.getRotation());

    // robot pose
    response.start_pose.x = x_robot;
    response.start_pose.y = y_robot;
    response.start_pose.theta = theta_robot;

    // Service call for latest occupancy grid map
    nav_msgs::GetMap srv;

    if (get_map_prob_client_.call(srv)) {
        ROS_INFO("OccupancyGrid map call successfull");
    } else {
        ROS_ERROR("Failed to call OccupancyGrid map");
    }

    latest_map_msg_ = srv.response.map;

    header = latest_map_msg_.header.frame_id;
    map_resolution_ = latest_map_msg_.info.resolution;
    map_width_ = latest_map_msg_.info.width;
    map_height_ = latest_map_msg_.info.height;

    x_offset = latest_map_msg_.info.origin.position.x;
    y_offset = latest_map_msg_.info.origin.position.y;

    // Update map with freespace threshold
    for (unsigned int i = 0; i < latest_map_msg_.data.size(); i++) {
        /*if (int(latest_map_msg_.data[i]) <= 25 && int(latest_map_msg_.data[i]) != -1) {
            latest_map_msg_.data[i] = 0;
        }*/
        /* todo check what is best between prev and this or a mixture between the two*/
        if (int(latest_map_msg_.data[i] != -1)) {
            if (int(latest_map_msg_.data[i]) <= 50) {
                latest_map_msg_.data[i] = 0;
            } else { latest_map_msg_.data[i] = 100; }
        }

        //std::cout << "data:(" << int(latest_map_msg_.data[i]) << ")"<< std::endl;
    }

    // Get robot position in map indices
    int x_robot_cell = positionToMapIndex(x_robot, y_robot, x_offset, y_offset, map_resolution_)[0];
    int y_robot_cell = positionToMapIndex(x_robot, y_robot, x_offset, y_offset, map_resolution_)[1];

    // Print result
    std::cout << "Robot cell: (" << x_robot_cell << ", " << y_robot_cell << ")" << std::endl;

    // Init frontier and visited flag to keep record while searching for frontiers
    std::vector<bool> frontier_flag(map_width_ * map_height_, false);
    std::vector<bool> visited_flag(map_width_ * map_height_, false);

    // Init breadth first search queue object
    std::queue<unsigned int> breadth_first_search;


    // Init with robot pose as a starting point of the search.
    // Assumption: robot pose is a free space
    if (latest_map_msg_.data.at(x_robot_cell + y_robot_cell * map_width_) == -1) {
        if (latest_map_msg_.data.at(x_robot_cell + 10 + y_robot_cell * map_width_) == 0) {
            breadth_first_search.push(x_robot_cell + 10 + y_robot_cell * map_width_);
        } else if (latest_map_msg_.data.at(x_robot_cell + (y_robot_cell + 10) * map_width_) == 0) {
            breadth_first_search.push(x_robot_cell + (y_robot_cell + 10) * map_width_);
        } else {
            ROS_WARN(
                    "Frontier exploration starts at unknown space and has no free space in front! Change orientation!");
        }
    } else {
        breadth_first_search.push(x_robot_cell + y_robot_cell * map_width_);
    }
    visited_flag[breadth_first_search.front()] = true;

    std::cout << "breadth_first_search: (" << breadth_first_search.size() << ")" << std::endl;
    // Search for frontiers
    while (!breadth_first_search.empty()) {
        unsigned int id = breadth_first_search.front();
        breadth_first_search.pop();

        std::vector<unsigned int> neighbour_vec = neighbour4(id, map_width_, map_height_);
        //std::cout << "neighbour_vec: (" << neighbour_vec.size() << ")" << std::endl;
        for (unsigned int i = 0; i < neighbour_vec.size(); i++) {
            // Check if cell is free and has not been visited, then add to queue
            if (int(latest_map_msg_.data.at(neighbour_vec[i])) == 0 &&
                !visited_flag[neighbour_vec[i]]) {
                visited_flag[neighbour_vec[i]] = true;
                breadth_first_search.push(neighbour_vec[i]);
            }
                // Check if cell is unknown and has not been marked as frontier
            else if (!frontier_flag[neighbour_vec[i]] &&
                     int(latest_map_msg_.data.at(neighbour_vec[i])) == -1) {
                frontier_flag[neighbour_vec[i]] = true;

                // Get frontier centroid around current cell
                geometry_msgs::Pose2D frontier_centroid;
                if (getFrontierCentroid(neighbour_vec[i], frontier_flag, map_width_, map_height_,
                                        map_resolution_, frontier_centroid)) {
                    // Check if frontier is not on robot position(problem on initialisation)
                    if (abs(frontier_centroid.x - x_robot) < 0.8 &&
                        abs(frontier_centroid.y - y_robot) < 0.8) {}
                    else {
                        response.frontier_list.push_back(frontier_centroid);
                    }
                }
            }
        }
    }
    //std::cout << "breadth_first_search[i]: (" << breadth_first_search.size()<< ")" << std::endl;

    // Generate GridCells msg of frontier centroids
    nav_msgs::GridCells frontier_points_msg;
    frontier_points_msg.header.frame_id = "map";
    frontier_points_msg.cell_width = 0.1;
    frontier_points_msg.cell_height = 0.1;
    for (unsigned int i = 0; i < response.frontier_list.size(); i++) {
        geometry_msgs::Point point;
        point.x = response.frontier_list[i].x;
        point.y = response.frontier_list[i].y;
        point.z = 0;
        frontier_points_msg.cells.push_back(point);
    }

    // Publish GridCells msg of frontier centroids
    frontier_cell_pub_.publish(frontier_points_msg);

    return true;
}

bool FrontierExploration::getFrontierCentroid(unsigned int initial_cell,
                                              std::vector<bool> &frontier_flag,
                                              int map_width,
                                              int map_height,
                                              float map_resolution,
                                              geometry_msgs::Pose2D &centroid) {

    geometry_msgs::Pose2D centroid_pose2D;
    int size = 1;
    double centroid_x, centroid_y, centroid_theta;
    std::vector<double> cxs;
    std::vector<double> cys;

    // Get x,y world coordinates of initial cell id and init centroid values
    double ix, iy, itheta;
    idToWorldXY(initial_cell, ix, iy, itheta, map_width, map_height, map_resolution, x_robot, y_robot, x_offset,
                y_offset);
    centroid_x = ix;
    centroid_y = iy;
    centroid_theta = itheta;

    // Init breadth first search queue
    std::queue<unsigned int> breadth_first_search;
    breadth_first_search.push(initial_cell);

    // Search for frontiers
    while (!breadth_first_search.empty()) {
        unsigned int id = breadth_first_search.front();
        breadth_first_search.pop();

        std::vector<unsigned int> neighbour8_vec = neighbour4(id, map_width, map_height);
        for (unsigned int i = 0; i < neighbour8_vec.size(); i++) {
            // Check if cell is unknown and has not been marked as a frontier cell
            if (int(latest_map_msg_.data.at(neighbour8_vec[i])) == -1 &&
                !frontier_flag[neighbour8_vec[i]]) {
                bool has_free_neighbour4 = false;
                std::vector<unsigned int> neighbour4_vec = neighbour8(neighbour8_vec[i], map_width, map_height);
                // Check if a neighbour is a free cell
                for (unsigned int j = 0; j < neighbour4_vec.size(); j++) {
                    if (int(latest_map_msg_.data.at(neighbour4_vec[j])) == 0) {
                        has_free_neighbour4 = true;
                    }
                }
                // if a neighbour is a free cell, mark as frontier and update centroid
                if (has_free_neighbour4) {
                    frontier_flag[neighbour8_vec[i]] = true;

                    double new_x, new_y, new_theta;
                    idToWorldXY(neighbour8_vec[i], new_x, new_y, new_theta, map_width, map_height, map_resolution,
                                x_robot, y_robot, x_offset, y_offset);
                    //        ROS_INFO_STREAM("nx " << new_x << " ny " << new_y);
                    centroid_x += new_x;
                    cxs.push_back(new_x);
                    cys.push_back(new_y);
                    centroid_y += new_y;
                    //centroid_theta += new_theta;
                    centroid_theta = new_theta;
                    size++;

                    breadth_first_search.push(neighbour8_vec[i]);
                }
            }
        }
    }

    //std::cout << "size: (" << size << ")" << std::endl;

    // Calculate centroid pose
    centroid_x /= size;
    centroid_y /= size;
    //centroid_theta /= size;
    centroid.x = centroid_x;
    centroid.y = centroid_y;
    centroid.theta = centroid_theta;
    ROS_INFO_STREAM("CENTROID " << centroid_x << " " << centroid_y);

    std::sort(cxs.begin(), cxs.end());
    std::sort(cys.begin(), cys.end());
//    if (cxs.size() > 0) {
//        size_t n = cxs.size() / 2;
//        nth_element(cxs.begin(), cxs.begin() + n, cxs.end());
//        ROS_INFO_STREAM(cxs[n]);
//
//        n = cxs.size() / 2;
//        nth_element(cys.begin(), cys.begin() + n, cys.end());
//        ROS_INFO_STREAM(cys[n]);
//        centroid.x = cxs[n];
//        centroid.y = cys[n];
//    }

    // Check if size of frontier is big enough and if a neighbour is a wall
    if (size > frontier_size_) {
        std::vector<int> cell = positionToMapIndex(centroid_x, centroid_y, x_offset,
                                                   y_offset, map_resolution);
        unsigned int id = cell[0] + cell[1] * map_width;
        std::vector<unsigned int> neighbour_x_cells = neighbourXCells(id, map_width, map_height, 8);
        for (unsigned int i = 0; i < neighbour_x_cells.size(); i++) {
            if (int(latest_map_msg_.data.at(neighbour_x_cells[i])) > 90) {
//                ROS_INFO_STREAM("NOK bad position but size is " << size << " \n ");
//                {
//                    double x_1 = (cxs.at(0) + centroid_x) / 2;
//                    double x_2 = (cxs.at(cxs.size() - 1) + centroid_x) / 2;
//                    double y_1 = (cys.at(0) + centroid_y) / 2;
//                    double y_2 = (cxs.at(cys.size() - 1) + centroid_y) / 2;
//                    std::vector<int> possibilities;
//
//                    std::vector<int> cell_tmp = positionToMapIndex(x_1, y_1, x_offset, y_offset, map_resolution);
//                    unsigned int id_tmp = cell_tmp[0] + cell_tmp[1] * map_width;
//                    if (int(latest_map_msg_.data.at(id_tmp)) >= 0) {
//                        possibilities.push_back(id_tmp);
//                    }
//
//                    cell_tmp = positionToMapIndex(x_1, y_2, x_offset, y_offset, map_resolution);
//                    id_tmp = cell_tmp[0] + cell_tmp[1] * map_width;
//                    if (int(latest_map_msg_.data.at(id_tmp)) >= 0) {
//                        possibilities.push_back(id_tmp);
//                    }
//                    cell_tmp = positionToMapIndex(x_2, y_2, x_offset, y_offset, map_resolution);
//                    id_tmp = cell_tmp[0] + cell_tmp[1] * map_width;
//                    if (int(latest_map_msg_.data.at(id_tmp)) >= 0) {
//                        possibilities.push_back(id_tmp);
//                    }
//                    cell_tmp = positionToMapIndex(x_2, y_1, x_offset, y_offset, map_resolution);
//                    id_tmp = cell_tmp[0] + cell_tmp[1] * map_width;
//                    if (int(latest_map_msg_.data.at(id_tmp)) >= 0) {
//                        possibilities.push_back(id_tmp);
//                    }
//
//                    double dist = 10000;
//                    bool found = false;
//                    double saved_x,saved_y;
//                    for (auto j : possibilities) {
//                        ROS_INFO_STREAM("TRY");
//                        neighbour_x_cells = neighbourXCells(j, map_width, map_height, 8);
//                        bool failed = false;
//                        for (unsigned int k = 0; k < neighbour_x_cells.size(); k++) {
//                            if (int(latest_map_msg_.data.at(neighbour_x_cells[k])) > 90) {
//                                failed = true;
//                            }
//                        }
//                        if (!failed) {
//                            double x;
//                            double y;
//                            double theta;
//                            idToWorldXY(j, x, y, theta, map_width, map_height, map_resolution, x_robot, y_robot,
//                                        x_offset, y_offset);
//                            double tmp_dist = std::sqrt(
//                                    (x - centroid_x) * (x - centroid_x) + (y - centroid_y) * (y - centroid_y));
//                            if (tmp_dist < dist) {
//                                dist = tmp_dist;
//                                saved_x = x;
//                                saved_y = y;
//                                found = true;
//                                ROS_INFO_STREAM("yay");
//                            }
//                        }
//                    }
//                    if (found) {
//                        centroid.x = saved_x;
//                        centroid.y = saved_y;
//                        return true;
//                    }
//                }
                double dist = 1000;
                bool found = false;
                double saved_x,saved_y;
                for (int j = 0; j < cxs.size(); j++) {
                    cell = positionToMapIndex(cxs[j], cys[j], x_offset,
                                              y_offset, map_resolution);
                    id = cell[0] + cell[1] * map_width;
                    neighbour_x_cells = neighbourXCells(id, map_width, map_height, 8);
                    bool hit = false;
                    for (unsigned int k = 0; k < neighbour_x_cells.size(); k++) {
                        if (int(latest_map_msg_.data.at(neighbour_x_cells[k])) > 90) {
                            hit = true;
                            break;
                        }
                    }
                    if (!hit) {
                        double tmp_dist = std::sqrt(
                                (cxs[j] - centroid_x) * (cxs[j] - centroid_x) + (cys[j] - centroid_y) * (cys[j] - centroid_y));
                        if (tmp_dist < dist) {
                            dist = tmp_dist;
                            saved_x = cxs[j];
                            saved_y = cys[j];
                            found = true;
                        }
                    }
                }
                if (!found)
                    return false;
                else {
                    centroid.x = saved_x;
                    centroid.y = saved_y;
                    return true;
                }
            }
        }
        return true;
    } else {
        ROS_INFO_STREAM("NOK small size \n ");
        return false;
    }
}


void FrontierExploration::idToWorldXY(unsigned int id, double &x, double &y, double &theta,
                                      int map_width, int map_height, float map_resolution, double x_start,
                                      double y_start, double x_off, double y_off) {
    unsigned int iy_cell = id / map_width;
    unsigned int ix_cell = id - iy_cell * map_width;
    x = int(ix_cell) * map_resolution + x_off + map_resolution / 2.0;
    y = int(iy_cell) * map_resolution + y_off + map_resolution / 2.0;
    x_diff = x - x_start;
    y_diff = y - y_start;
    theta = xyDiffToYaw(x_diff, y_diff);
}

std::vector<unsigned int> FrontierExploration::neighbourXCells(unsigned int id,
                                                               int map_width, int map_height, unsigned int n_cells) {
    std::vector<unsigned int> neighbour_vec;

    int br_corner = id - n_cells - n_cells * map_width;
    for (unsigned int i = 0; i <= 2 * n_cells; i++) {
        for (unsigned int j = 0; j <= 2 * n_cells; j++) {
            int new_id = br_corner + j + i * map_width;
            if (new_id >= 0 && new_id <= map_width * map_height && new_id != int(id)) {
                neighbour_vec.push_back(new_id);
            }
        }
    }

    return neighbour_vec;
}

std::vector<unsigned int> FrontierExploration::neighbour8(unsigned int id,
                                                          int map_width, int map_height) {
    std::vector<unsigned int> neighbour8_vec;
    int id_int = int(id);

    if (id_int >= map_width) {
        neighbour8_vec.push_back(id_int - map_width);
        if (id_int % map_width > 0) {
            neighbour8_vec.push_back(id_int - 1);
            neighbour8_vec.push_back(id_int - map_width - 1);
        }
        if (id_int % map_width < map_width - 1) {
            neighbour8_vec.push_back(id_int + 1);
            neighbour8_vec.push_back(id_int - map_width + 1);
        }
    }
    if (id_int < map_width * (map_height - 1)) {
        neighbour8_vec.push_back(id_int + map_width);
        if (id_int % map_width > 0) {
            neighbour8_vec.push_back(id_int + map_width - 1);
        }
        if (id_int % map_width < map_width - 1) {
            neighbour8_vec.push_back(id_int + map_width + 1);
        }
    }

    return neighbour8_vec;
}

std::vector<unsigned int> FrontierExploration::neighbour4(unsigned int id,
                                                          int map_width, int map_height) {
    std::vector<unsigned int> neighbour4_vec;
    int id_int = int(id);

    if (id_int >= map_width) {
        neighbour4_vec.push_back(id_int - map_width);
    }
    if (id_int < map_width * (map_height - 1)) {
        neighbour4_vec.push_back(id_int + map_width);
    }
    if (id_int % map_width > 0) {
        neighbour4_vec.push_back(id_int - 1);
    }
    if (id_int % map_width < map_width - 1) {
        neighbour4_vec.push_back(id_int + 1);
    }
    return neighbour4_vec;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "frontier_exploration");
    ros::NodeHandle nh;
    FrontierExploration frontier_exploration(nh);
    ros::spin();
    return 0;
}