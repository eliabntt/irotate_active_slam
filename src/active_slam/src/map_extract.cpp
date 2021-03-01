#include <map_extract.h>
#include <visualization_msgs/Marker.h>
#include <numeric>

/**
 * prerequisite:
 * roslaunch robotino_simulation world.launch
 * roslaunch robotino_simulaiton rtabmap.launch delete:=-d
 * roslauch active_slam map_extract.launch
 * rosservice call /map_grid_cell_service  
 *  "pose_input:
 *   x: 0.0
 *   y: 0.0
 *   theta: 0.0
 *   fov_angle_deg: 100
 *   kind_cost: 0"
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
 *  function which creates map ids from index.
 */
int mapIndexToId(int x_index, int y_index, int width) {
    int id = x_index + y_index * width;
    return id;
}

/**
* function which provides id to position.
* id = x+y*width
*/
std::vector<double> idToPosition(unsigned int id,
                                 int map_width, int map_height, float map_resolution, double x_off, double y_off) {
    unsigned int iy_cell = id / map_width;
    unsigned int ix_cell = id - iy_cell * map_width;
    std::vector<double> position_cell(2);
    position_cell[0] = int(ix_cell) * map_resolution + x_off + map_resolution / 2.0;
    position_cell[1] = int(iy_cell) * map_resolution + y_off + map_resolution / 2.0;

    return position_cell;
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

/**
* startAngle: start orientation
* angle: angle that will be checked
* fov_angle_rad: angle between this range
*/
bool is_angle_between(double startAngle, double angle, double fov_angle_rad) {
    double anglediff = fmod((startAngle - angle + M_PI + 2 * M_PI), (2 * M_PI)) - M_PI;
    if (anglediff <= (fov_angle_rad / 2) && anglediff >= -(fov_angle_rad / 2)) {
        return true;
    } else {
        return false;
    }
}

bool is_angle_between_cell(double angle_center, double startAngle, double occ_min_angle_rad, double occ_max_angle_rad) {
    double fov_cell = abs(fmod((occ_max_angle_rad - occ_min_angle_rad + M_PI + 2 * M_PI), (2 * M_PI)) - M_PI);
    fov_cell -= 0.01 * M_PI / 180;
    return (is_angle_between(startAngle, angle_center, fov_cell));
}

// Function to calculate distance
double distance(double x_origin_fov, double y_origin_fov, double x_point, double y_point) {
    // Calculating distance
    return sqrt(pow(x_point - x_origin_fov, 2) +
                pow(y_point - y_origin_fov, 2) * 1.0);
}

enum COST_TYPE {
    NUMBER_OF_CELLS, //0
    BASELINE, //1
    CUSTOM_PROB, //2
    CUSTOM_PROB_WEIGHT_STATIC, //3
    CUSTOM_PROB_WEIGHT_DYNAMIC, //4
    CUSTOM_PROB_WEIGHT_DYNAMIC_AND_OBS, //5
    RANDOM, //6
    INTERPOLATION, //7
    CUSTOM_PROB_WEIGHT_STATIC_AND_OBS, // 8
    CUSTOM_PROB_AND_OBS // 9
};

double MapExtract::cost_function(const int &kind_cost, const int &prob, const double &dist, const double &length) {
    // exp weight
    double lambda = 0.25;
    double weight = std::exp(-lambda * dist);
    // convert prob [-1] U [0:100] to [0,1]. -1(unexplored) => 0.5
    double pj = (prob == -1 ? 0.5 : prob / 100.0);

    if (kind_cost == COST_TYPE::NUMBER_OF_CELLS) {
        // just count the cell (discounted)
        return 1 * weight;
    } else {
        // amount of information.
        if (pj == 0) pj = 0.00000000001;
        if (pj == 1) pj = 0.99999999999;
        double shannon = -((1 - pj) * std::log2(1 - pj) + pj * std::log2(pj));

        double dist_to_frontier = (length - dist);

        // We use 3.1 to avoid roundings.
        // 3.1 is the midpoint between 1.5 and 4.5.
        // 4.5 is where the dynamic start going toward exploration.
        // 1.5 is the saturation point for exploration (weight coverage = 0.2)
        double k_static = (dist_to_frontier <= 1.5) ? 0.2 : 0.8;

        // dynamic change of k. 0.8 is max, 0.2 is min value. Linear decrease of weight
        // (0.8*exp(0.8*x) +0.2*exp(6/x))/(exp(0.8*x) + exp(6/x))
        double k_dynamic = std::max(0.2, std::min(0.8, 0.2 * dist_to_frontier - 0.1));

//        if (prob > 0 && prob < 100) {
//            ROS_INFO_STREAM("check prob: " << prob);
//            ROS_INFO_STREAM("check p: " << pj);
//            ROS_INFO_STREAM("check dist: " << dist);
//            ROS_INFO_STREAM("check length: " << length);
//            ROS_INFO_STREAM("check shannon: " << shannon);
//            ROS_INFO_STREAM("check threshold dist: " << dist_to_frontier);
//            ROS_INFO_STREAM("check k_dynamic: " << k_dynamic);
//            ROS_INFO_STREAM("check k_static: " << k_static);
//            ROS_INFO_STREAM("check base: " << weight * (prob == -1 ? 1 : (1 - pj)));
//            ROS_INFO_STREAM("check custom_prob: " << weight * shannon);
//            ROS_INFO_STREAM("check k_const: " << weight * shannon * (prob == -1 ? (1 - k_static) : k_static));
//            ROS_INFO_STREAM("check k_dynamic: " << weight * shannon * (prob == -1 ? (1 - k_dynamic) : k_dynamic));
//            ROS_INFO_STREAM("-----------check finished-----------");
//        }
        if (kind_cost == COST_TYPE::BASELINE) {
            // new cells (prob == -1) are weighted 1. Everything else is counted as 1-pj. Vj is omitted for simplicity. e^-lambda is common
            return weight * (prob == -1 ? 1 : (1 - pj));
        } else if (kind_cost == COST_TYPE::CUSTOM_PROB || kind_cost == COST_TYPE::RANDOM ||
                   kind_cost == COST_TYPE::INTERPOLATION) {
            return weight * shannon;
        } else if (kind_cost == COST_TYPE::CUSTOM_PROB_WEIGHT_STATIC) {
            return weight * shannon * (prob == -1 ? (1 - k_static) : k_static);
        } else if (kind_cost == COST_TYPE::CUSTOM_PROB_WEIGHT_DYNAMIC) {
            return weight * shannon * (prob == -1 ? (1 - k_dynamic) : k_dynamic);
        } else if (kind_cost == COST_TYPE::CUSTOM_PROB_WEIGHT_DYNAMIC_AND_OBS) {
            return weight * shannon * (prob == -1 ? (1 - k_dynamic) : k_dynamic) +
                   weight * (prob >= prob_threshold_free_ / 100.0 ? 1 : 0);
        } else if (kind_cost == COST_TYPE::CUSTOM_PROB_WEIGHT_STATIC_AND_OBS) {
            return weight * shannon * (prob == -1 ? (1 - k_static) : k_static) +
                   weight * (prob >= prob_threshold_free_ / 100.0 ? 1 : 0);
        } else if (kind_cost == COST_TYPE::CUSTOM_PROB_AND_OBS) {
            return weight * shannon + weight * (prob >= prob_threshold_free_ / 100.0 ? 1 : 0);
        } else {
            ROS_WARN_STREAM("Wrong cost type");
            return 0;
        }
    }
}

/**
 * Check if cells are in the region of the field of view (fov).
 */
bool checkPoint(double radius, double x, double y, double x_start, double y_start, double angle_fov,
                double startAngle, double min_distance = 0) {
    // Calculate polar co-ordinates
    double x_diff = x - x_start;
    double y_diff = y - y_start;
    double polarradius = sqrt(x_diff * x_diff + y_diff * y_diff);
    double cell_angle = xyDiffToYaw(x_diff, y_diff);

    // check if cell is within FOV (radius, angle)
    if (polarradius <= radius && polarradius >= min_distance) {
        return is_angle_between(cell_angle, startAngle, angle_fov);
    }
    return false;
}

double getTheta(geometry_msgs::Quaternion msg) {
    tf::Quaternion q(msg.x, msg.y, msg.z, msg.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw = yaw * 180 / M_PI;
    yaw > 360 ? yaw -= 360 : (yaw < 0 ? yaw += 360 : yaw);
    return yaw;
}


MapExtract::MapExtract(ros::NodeHandle nh, ros::NodeHandle private_nh) :
        nh_(nh), private_nh_(private_nh) {
    ROS_INFO("Started MapExtract");
    initParams();
}

MapExtract::~MapExtract() {}

void MapExtract::initParams() {

    // Get ros param
    private_nh_.param<double>("depth", radius_camera_, 4);
    private_nh_.param<int>("prob_threshold_free", prob_threshold_free_, 30);
    private_nh_.param<int>("fov", fov_camera_deg_, 86);
    private_nh_.param<bool>("debug", debug_, false);
    private_nh_.param<double>("min_dist", min_distance_, 0.5);
    private_nh_.param<bool>("weighted_avg",weighted_avg, true);

//Publisher
    if (debug_) {
        // cell within FOV - before raytracing
        occ_cell_pub_ = nh_.advertise<nav_msgs::GridCells>("/occ_points_grid_cell", 1);
        free_cell_pub_ = nh_.advertise<nav_msgs::GridCells>("/free_points_grid_cell", 1);
        unk_cell_pub_ = nh_.advertise<nav_msgs::GridCells>("/unk_points_grid_cell", 1);

        // raytrace
        raytracing_target_pub_ = nh_.advertise<nav_msgs::GridCells>("/raytracing_target", 1);
        raytracing_occu_pub_ = nh_.advertise<nav_msgs::GridCells>("/raytracing_occu", 1);
        raytracing_free_pub_ = nh_.advertise<nav_msgs::GridCells>("/raytracing_free", 1);
        raytracing_unk_pub_ = nh_.advertise<nav_msgs::GridCells>("/raytracing_unk", 1);

        // origin of he FOV + BB
        origin_fov_pub_ = nh_.advertise<nav_msgs::GridCells>("/cell", 1);
        submap_bounding_box_cell_pub_ = nh_.advertise<nav_msgs::GridCells>("submap_bounding_box_grid_cell", 1);

    }
    opt_heading_pub_ = nh_.advertise<nav_msgs::GridCells>("opt_heading_cells", 1);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Init Client -
    get_map_prob_client_ = nh_.serviceClient<nav_msgs::GetMap>("/rtabmap/get_prob_map");

// Init Service
    set_path_service_ = nh_.advertiseService("get_best_path", &MapExtract::getBestPathServiceCallback,
                                             this);
    set_point_service_ = nh_.advertiseService("get_best_head", &MapExtract::getBestHeadServiceCallback,
                                              this);
    ROS_INFO("get_best_path started!");

}

void MapExtract::getBoundingBox(const double &theta, const double &fov, const double &x_start, const double &y_start,
                                double &x_max, double &x_min, double &y_max, double &y_min) {
    // create a minimum bounding box for this node
    x_max = max({x_start + radius_camera_ * cos(theta + fov / 2), x_start + radius_camera_ * cos(theta - fov / 2),
                 x_start});
    x_min = min({x_start + radius_camera_ * cos(theta + fov / 2), x_start + radius_camera_ * cos(theta - fov / 2),
                 x_start});
    y_max = max({y_start + radius_camera_ * sin(theta + fov / 2), y_start + radius_camera_ * sin(theta - fov / 2),
                 y_start});
    y_min = min({y_start + radius_camera_ * sin(theta + fov / 2), y_start + radius_camera_ * sin(theta - fov / 2),
                 y_start});
    is_zero_inside = false;
    if (is_angle_between(theta, 0, fov)) {
        x_max = x_start + radius_camera_;
        is_zero_inside = true;
    }
    // first quadrant - second quadrant
    if (is_angle_between(theta, M_PI_2, fov)) {
        y_max = y_start + radius_camera_;
    }
    // second quadrant - third quadrant
    if (is_angle_between(theta, M_PI, fov)) {
        x_min = x_start - radius_camera_;
    }
    // third quadrant - fourth quadrant
    if (is_angle_between(theta, -M_PI_2, fov)) {
        y_min = y_start - radius_camera_;
    }
}

void MapExtract::cellInFov(double const &theta_start, double const &fov, double const &x_start, double const &y_start,
                           std::vector<std::pair<double, double>> &occuVector, // angle, distance
                           std::vector<std::pair<double, double>> &min_max_angle_occu_vec, // min and max angles for each occupied cell
                           std::vector<std::pair<double, double>> &cellPos, // x,y
                           std::vector<std::pair<double, double>> &min_max_angle_vec,
                           std::vector<std::pair<double, double>> &cellVector,
                           std::vector<int> &prob_vec, const std::vector<bool> &added_before) {
    // fourth quadrant - first quadrant
    // in zero we have a rolling problem during computation of optimal heading since we move ourself in [0,360]
    geometry_msgs::Point point;

    double x_max, y_max, x_min, y_min;
    getBoundingBox(theta_start, fov, x_start, y_start, x_max, x_min, y_max, y_min);

    if (debug_)
        publish_bb(x_max, x_min, y_max, y_min, x_start, y_start, map_resolution_);

    prepare_messages(map_resolution_);

    std::vector<int> top_left_cell = positionToMapIndex(x_min, y_max, x_origin, y_origin, map_resolution_);
    std::vector<int> top_right_cell = positionToMapIndex(x_max, y_max, x_origin, y_origin, map_resolution_);
    std::vector<int> bottom_left_cell = positionToMapIndex(x_min, y_min, x_origin, y_origin, map_resolution_);
    std::vector<int> bottom_right_cell = positionToMapIndex(x_max, y_min, x_origin, y_origin, map_resolution_);

    for (int i = bottom_left_cell[0]; i <= top_right_cell[0]; i++) {
        for (int j = bottom_left_cell[1]; j <= top_right_cell[1]; j++) {
            int submap_id = mapIndexToId(i, j, map_width_);

            std::vector<double> cell_pos = idToPosition(submap_id, map_width_, map_height_, map_resolution_,
                                                        x_origin, y_origin);

            bool check_cell = checkPoint(radius_camera_, cell_pos[0], cell_pos[1], x_start, y_start, fov, theta_start);

            if (check_cell &&
                (!added_before.at(submap_id) || int(latest_map_msg_.data.at(submap_id)) > prob_threshold_free_)) {

                double x_diff_cell = cell_pos[0] - x_start;
                double y_diff_cell = cell_pos[1] - y_start;

                double point_distance = distance(x_start, y_start, cell_pos[0], cell_pos[1]);

                double angle_cell = xyDiffToYaw(x_diff_cell, y_diff_cell);

                double x_max_cell = cell_pos[0] + (map_resolution_ / 2);
                double x_min_cell = cell_pos[0] - (map_resolution_ / 2);
                double y_max_cell = cell_pos[1] + (map_resolution_ / 2);
                double y_min_cell = cell_pos[1] - (map_resolution_ / 2);

                double min_angle = min({xyDiffToYaw(x_max_cell - x_start, y_min_cell - y_start),
                                        xyDiffToYaw(x_max_cell - x_start, y_max_cell - y_start),
                                        xyDiffToYaw(x_min_cell - x_start, y_max_cell - y_start),
                                        xyDiffToYaw(x_min_cell - x_start, y_min_cell - y_start)});
                double max_angle = max({xyDiffToYaw(x_min_cell - x_start, y_max_cell - y_start),
                                        xyDiffToYaw(x_min_cell - x_start, y_min_cell - y_start),
                                        xyDiffToYaw(x_max_cell - x_start, y_min_cell - y_start),
                                        xyDiffToYaw(x_max_cell - x_start, y_max_cell - y_start)});

                cellPos.emplace_back(std::make_pair(cell_pos[0], cell_pos[1]));
                cellVector.emplace_back(std::make_pair(angle_cell, point_distance));
                min_max_angle_vec.emplace_back(std::make_pair(min_angle, max_angle));

                prob_vec.push_back(int(latest_map_msg_.data.at(submap_id)));

                // count occupied fov
                if (prob_vec.at(prob_vec.size() - 1) > prob_threshold_free_ &&
                    prob_vec.at(prob_vec.size() - 1) != -1) {
                    if (debug_) {
                        // Generate GridCells msg of occupied cells in fov
                        occ_points_msg.cells.push_back(point);
                    }
                    occuVector.emplace_back(std::make_pair(angle_cell, point_distance));
                    min_max_angle_occu_vec.emplace_back(std::make_pair(min_angle, max_angle));
                }

                if (debug_) {
                    if (prob_vec.at(prob_vec.size() - 1) == -1) {
                        // Generate GridCells msg of unknown cells in fov
                        unk_points_msg.cells.push_back(point);
                    } else if (prob_vec.at(prob_vec.size() - 1) <= prob_threshold_free_) {
                        // Generate GridCells msg of free cells in fov
                        free_points_msg.cells.push_back(point);
                    }
                }
            }
        }
    }
}

template<typename T, typename Compare>
std::vector<std::size_t> sort_permutation(
        const std::vector<T> &vec,
        Compare &compare) {
    std::vector<std::size_t> p(vec.size());
    std::iota(p.begin(), p.end(), 0);
    std::sort(p.begin(), p.end(),
              [&](std::size_t i, std::size_t j) { return compare(vec[i], vec[j]); });
    return p;
}

template<typename T>
std::vector<T> apply_permutation(
        const std::vector<T> &vec,
        const std::vector<std::size_t> &p) {
    std::vector<T> sorted_vec(vec.size());
    std::transform(p.begin(), p.end(), sorted_vec.begin(),
                   [&](std::size_t i) { return vec[i]; });
    return sorted_vec;
}

bool sortasc(const std::pair<double, double> &a,
             const std::pair<double, double> &b) {
    return (a.second < b.second);
}

bool sortdesc(const std::pair<double, double> &a,
              const std::pair<double, double> &b) {
    return (a.second > b.second);
}

void MapExtract::raytracing(std::vector<std::pair<double, double>> &occuVector, // angle, distance
                            std::vector<std::pair<double, double>> &min_max_angle_occu_vec, // min and max angles for each occupied cell
                            const std::vector<std::pair<double, double>> &cellPos, // x,y
                            const std::vector<std::pair<double, double>> &min_max_angle_vec,
                            std::vector<std::pair<double, double>> &cellVector,
                            const std::vector<int> &prob_vec,
                            std::vector<std::pair<int, int>> &visible_cells,
                            std::vector<std::pair<double, double>> &visibleCellsPos,
                            const std::vector<bool> &added_before) {

    geometry_msgs::Point point;

    std::vector<std::pair<double, double>> occu;
    std::vector<std::pair<double, double>> min_max_angle_occu_vec2;
    for (int i = 0; i < occuVector.size(); i++) {
        bool max_inside = false;
        bool min_inside = false;
        for (int j = 0; j < occuVector.size(); j++) {
            if (occuVector.at(j).second < occuVector.at(i).second) {
                if (!max_inside &&
                    is_angle_between_cell(occuVector.at(j).first, min_max_angle_occu_vec.at(i).second,
                                          min_max_angle_occu_vec.at(j).first + 0.001,
                                          min_max_angle_occu_vec.at(j).second - 0.001)) {
                    max_inside = true;
                }
                if (!min_inside && is_angle_between_cell(occuVector.at(j).first, min_max_angle_occu_vec.at(i).first,
                                                         min_max_angle_occu_vec.at(j).first + 0.001,
                                                         min_max_angle_occu_vec.at(j).second - 0.001)) {
                    min_inside = true;
                }
                if (max_inside && min_inside)
                    break;
            }
        }
        if ((!max_inside || !min_inside)) {
            occu.push_back(occuVector.at(i));
            min_max_angle_occu_vec2.push_back(min_max_angle_occu_vec.at(i));
        }
    }

    auto p = sort_permutation(occu, sortasc);
    occu = apply_permutation(occu, p);
    min_max_angle_occu_vec2 = apply_permutation(min_max_angle_occu_vec2, p);

//    ROS_INFO_STREAM("s " << occu.size());
//    ROS_INFO_STREAM("s1 " << occuVector.size());

    for (int i = 0; i < cellVector.size(); i++) {
        bool max_inside = false;
        bool min_inside = false;
        bool non_visible = (cellVector.at(i).second <= min_distance_);

        if (!non_visible) {
            if (!occu.empty() && occu.at(0).second < cellVector.at(i).second)
                for (int j = 0; j < occu.size(); j++) {
                    if (occu.at(j).second < cellVector.at(i).second) {
                        if (!max_inside && is_angle_between_cell(occu.at(j).first, min_max_angle_vec.at(i).second,
                                                                 min_max_angle_occu_vec2.at(j).first,
                                                                 min_max_angle_occu_vec2.at(j).second)) {
                            max_inside = true;
                        }
                        if (!min_inside && is_angle_between_cell(occu.at(j).first, min_max_angle_vec.at(i).first,
                                                                 min_max_angle_occu_vec2.at(j).first,
                                                                 min_max_angle_occu_vec2.at(j).second)) {
                            min_inside = true;
                        }
                        if (max_inside && min_inside)
                            break;
                    } else {
                        break;
                    }
                }
        }


        point.x = cellPos.at(i).first;
        point.y = cellPos.at(i).second;
        point.z = 0;

        std::vector<int> submap_id = positionToMapIndex(cellPos.at(i).first, cellPos.at(i).second, x_origin, y_origin,
                                                        map_resolution_);
        int id = mapIndexToId(submap_id.at(0), submap_id.at(1), map_width_);

        // cells visible
        if (!non_visible && ((!max_inside && !min_inside && prob_vec.at(i) < prob_threshold_free_) ||
                             (!max_inside || !min_inside && prob_vec.at(i) >= prob_threshold_free_)) &&
            !added_before.at(id)) {

            if (cellVector.at(i).first <= 0) {
                cellVector.at(i).first = cellVector.at(i).first + 2 * M_PI;
            }
            visible_cells.emplace_back(
                    std::make_pair(round(cellVector.at(i).first * (180 / M_PI)), prob_vec.at(i)));

            if (is_zero_inside) {
                if (cellVector.at(i).first < fov_camera_rad) {
                    visible_cells.emplace_back(
                            std::make_pair(round((cellVector.at(i).first + 2 * M_PI) * (180 / M_PI)),
                                           prob_vec.at(i)));
                }
            }

            visibleCellsPos.push_back(cellPos.at(i));

            if (debug_) {
                // unk
                if (prob_vec.at(i) == -1) {
                    raytracing_unk_msg.cells.push_back(point);
                }    // occu
                else if (prob_vec.at(i) > prob_threshold_free_) {
                    raytracing_occu_msg.cells.push_back(point);
                }
                    // free
                else {
                    raytracing_free_msg.cells.push_back(point);
                }
            }
        } else if (debug_) {
            raytracing_target_msg.cells.push_back(point);
        }
    }

    if (debug_) {
        // Publish GridCells msg of occupied cells in fov
        occ_cell_pub_.publish(occ_points_msg);

        // Publish GridCells msg of free cells in fov
        free_cell_pub_.publish(free_points_msg);

        // Publish GridCells msg of unknown cells in fov
        unk_cell_pub_.publish(unk_points_msg);

        // Publish GridCells msg of unknown cells in fov
        raytracing_target_pub_.publish(raytracing_target_msg);

        // Publish GridCells msg of unknown cells in fov
        raytracing_occu_pub_.publish(raytracing_occu_msg);

        // Publish GridCells msg of unknown cells in fov
        raytracing_free_pub_.publish(raytracing_free_msg);

        // Publish GridCells msg of unknown cells in fov
        raytracing_unk_pub_.publish(raytracing_unk_msg);
    }
}

double MapExtract::getMaxUtility(std::vector<std::pair<int, int>> &visible_cells,
                                 const std::vector<std::pair<double, double>> &visibleCellsPos,
                                 const double &x_start, const double &y_start, std::vector<int> &max_utility_angles,
                                 const double &wp_dist, const double &path_length, const double &inter_angle) {
    sort(visible_cells.begin(), visible_cells.end());

    geometry_msgs::Point point;

    int max_angle_ = visible_cells.at(visible_cells.size() - 1).first;
    std::vector<std::pair<int, double>> cost_vec;

    for (auto &visible_cell : visible_cells) {
        int current_angle = visible_cell.first;
        double cost = cost_function(kind_cost_, visible_cell.second, wp_dist,
                                    path_length);

        bool angle_found = false;

        for (auto &j : cost_vec) {
            // update cost values
            if (j.first > visible_cell.first - fov_camera_deg_) {
                j.second = j.second + cost;
            }
            // angle of vector visible_cells is already in new vector
            if (j.first == visible_cell.first) {
                angle_found = true;
            }
        }
        // offset +1 because of the angle rounding
        if (!angle_found && current_angle <= max_angle_ - fov_camera_deg_ + 1) {
            cost_vec.emplace_back(std::make_pair(current_angle, cost));
        }
    }

    max_utility_angles.push_back(cost_vec.at(0).first);
    double max_cost_ = cost_vec.at(0).second;

    // todo if random or interpolation i -> random or s.t. dist(orientation, i + fov_camera_deg) is minimized
    if (kind_cost_ != COST_TYPE::RANDOM && kind_cost_ != COST_TYPE::INTERPOLATION) {
        for (int i = 1; i < cost_vec.size(); i++) {
            if (cost_vec.at(i).second > max_cost_) {
                max_cost_ = cost_vec.at(i).second;
                max_utility_angles.clear();
            }
            if (cost_vec.at(i).second == max_cost_) {
                max_utility_angles.push_back(cost_vec.at(i).first);
            }
        }
    } else {
        max_utility_angles.clear();
        if (kind_cost_ == COST_TYPE::RANDOM) {
            int i = rand() % cost_vec.size();
            max_cost_ = cost_vec.at(i).second;
            max_utility_angles.emplace_back(cost_vec.at(i).first);
        } else {
            // dist between what will be the center and the interpolated angle
            double dist =
                    fmod((cost_vec.at(0).first + fov_camera_deg_ / 2 - inter_angle + 180 + 2 * 180), (2 * 180)) - 180;
            if (dist < 0)
                dist += 360;
            for (int i = 1; i < cost_vec.size(); i++) {
                double current =
                        fmod((cost_vec.at(i).first + fov_camera_deg_ / 2 - inter_angle + 180 + 2 * 180), (2 * 180)) -
                        180;
                if (current < 0)
                    current += 360;
                if (current < dist) {
                    dist = current;
                    max_cost_ = cost_vec.at(i).second;
                    max_utility_angles.clear();
                    max_utility_angles.emplace_back(cost_vec.at(i).first);
                }
            }
        }
    }

    for (auto &i : max_utility_angles) {
        i += fov_camera_deg_ / 2;
        i > 360 ? i -= 360 : i;
    }
    return max_cost_;
}

double MapExtract::normalizeToResolution(const double &coord) {
    double res = coord;
    if (fmod(coord, map_resolution_) != 0) {
        if (fmod(coord, map_resolution_) > map_resolution_ / 2) {
            res = coord + (map_resolution_ - fmod(coord, map_resolution_));
        } else {
            res = coord - fmod(coord, map_resolution_);
        }
    }
    return res;
}

bool MapExtract::getMap() {
    // Service call for latest occupancy grid map
    nav_msgs::GetMap srv;
    if (get_map_prob_client_.call(srv)) {
        ROS_INFO("OccupancyGrid map call successfull");
    } else {
        ROS_ERROR("Failed to call OccupancyGrid map");
        return false;
    }
    latest_map_msg_ = srv.response.map;
    map_resolution_ = latest_map_msg_.info.resolution;
    map_width_ = latest_map_msg_.info.width;
    map_height_ = latest_map_msg_.info.height;
    x_origin = latest_map_msg_.info.origin.position.x;
    y_origin = latest_map_msg_.info.origin.position.y;
    return true;
}

bool MapExtract::getBestHeadServiceCallback(active_slam::get_best_head::Request &request,
                                            active_slam::get_best_head::Response &response) {
    // available fov span
    int fov_angle_deg_ = request.fov_angle_deg;
    // allow a pan angle of 1 deg for boundaries, limit to 360
    fov_angle_deg_ = std::min(fov_angle_deg_ + 1, 360);
    if (fov_angle_deg_ < fov_camera_deg_) {
        fov_angle_deg_ = fov_camera_deg_;
        ROS_WARN("fov angle is smaller than the fov of the camera. fov angle is set to the fov value of the camera");
    }
    double fov_angle_rad = fov_angle_deg_ * (M_PI / 180);

    // fov that can be seen by the cam
    fov_camera_rad = fov_camera_deg_ * (M_PI / 180);

    // type of cost function
    kind_cost_ = request.kind_cost;

    // get map
    if (!getMap()) {
        return false;
    }

    std::vector<int> all_costs = {COST_TYPE::NUMBER_OF_CELLS, COST_TYPE::BASELINE, COST_TYPE::CUSTOM_PROB,
                                  COST_TYPE::CUSTOM_PROB_WEIGHT_STATIC, COST_TYPE::CUSTOM_PROB_WEIGHT_DYNAMIC,
                                  COST_TYPE::CUSTOM_PROB_WEIGHT_DYNAMIC_AND_OBS, COST_TYPE::RANDOM,
                                  COST_TYPE::INTERPOLATION, COST_TYPE::CUSTOM_PROB_WEIGHT_STATIC_AND_OBS,
                                  COST_TYPE::CUSTOM_PROB_AND_OBS};
    std::vector<double> tot_utility;
    tot_utility.resize(all_costs.size(), 0);

    int wanted_kind = kind_cost_;

    double dist = 0;
    double x_start = normalizeToResolution(request.pose.position.x);
    double y_start = normalizeToResolution(request.pose.position.y);

    double theta_start_deg = request.theta;
    double theta_start_rad = theta_start_deg * M_PI / 180;

    std::vector<std::pair<double, double>> occuVector; // angle, distance
    std::vector<std::pair<double, double>> min_max_angle_occu_vec; // min and max angles for each occupied cell
    std::vector<std::pair<double, double>> cellPos; // x,y
    std::vector<std::pair<double, double>> min_max_angle_vec;
    std::vector<std::pair<double, double>> cellVector;
    std::vector<int> prob_vec;
    std::vector<bool> added_before;
    added_before.resize(latest_map_msg_.data.size(), false);

    cellInFov(theta_start_rad, fov_angle_rad, x_start, y_start,
              occuVector, min_max_angle_occu_vec, cellPos, min_max_angle_vec, cellVector, prob_vec, added_before);


    std::vector<std::pair<int, int>> visible_cells;
    std::vector<std::pair<double, double>> visibleCellsPos;
    raytracing(occuVector, min_max_angle_occu_vec, cellPos, min_max_angle_vec, cellVector, prob_vec,
               visible_cells, visibleCellsPos, added_before);

    double cur_head = theta_start_deg;

    clear_messages();
    for (auto cost:all_costs) {
        std::vector<int> max_utility_angles;
        kind_cost_ = cost;
        if (debug_ || cost == wanted_kind)
            tot_utility[cost] += getMaxUtility(visible_cells, visibleCellsPos, x_start, y_start,
                                               max_utility_angles,
                                               dist, request.length,
                                               cur_head);
        double min_diff = 360;
        double chosen_angle;
        for (auto i: max_utility_angles) {
            double angle_diff = i - cur_head;
            angle_diff < 0 ? angle_diff += 360 : angle_diff;
            angle_diff > 360 ? angle_diff -= 360 : angle_diff;
            if (angle_diff <= min_diff) {
                chosen_angle = i;
            }
        }

        if (debug_ || cost == wanted_kind) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time::now();
            marker.ns = std::to_string(cost);
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            marker.id = -1;

            marker.pose.position.x = x_start;
            marker.pose.position.y = y_start;
            marker.pose.position.z = 0;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(chosen_angle * M_PI / 180);

            marker.scale.x = 1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            if (cost == COST_TYPE::NUMBER_OF_CELLS) {
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
            } else if (cost == COST_TYPE::BASELINE) {
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
            } else if (cost == COST_TYPE::CUSTOM_PROB) {
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
            } else if (cost == COST_TYPE::CUSTOM_PROB_WEIGHT_STATIC) {
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
            } else if (cost == COST_TYPE::CUSTOM_PROB_WEIGHT_DYNAMIC) {
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
            } else if (cost == COST_TYPE::CUSTOM_PROB_WEIGHT_DYNAMIC_AND_OBS) {
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 1.0f;
            } else if (cost == COST_TYPE::RANDOM) {
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 1.0f;
            } else if (cost == COST_TYPE::INTERPOLATION) {
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
            } else if (cost == COST_TYPE::CUSTOM_PROB_WEIGHT_STATIC_AND_OBS) {
                marker.color.r = 0.2f;
                marker.color.g = 0.5f;
                marker.color.b = 0.8f;
            } else if (cost == COST_TYPE::CUSTOM_PROB_AND_OBS) {
                marker.color.r = 0.8f;
                marker.color.g = 0.5f;
                marker.color.b = 0.2f;
            }

            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();
            if (cost == wanted_kind)
                marker_pub.publish(marker);
        }

        if (cost == wanted_kind) {
            response.orientation = chosen_angle * M_PI / 180;
        }
    }
}

bool MapExtract::getBestPathServiceCallback(active_slam::get_best_path::Request &request,
                                            active_slam::get_best_path::Response &response) {

    ROS_INFO_STREAM("req received");
    // available fov span
    int fov_angle_deg_ = request.fov_angle_deg;
    // allow a pan angle of 1 deg for boundaries, limit to 360
    fov_angle_deg_ = std::min(fov_angle_deg_ + 1, 360);
    if (fov_angle_deg_ < fov_camera_deg_) {
        fov_angle_deg_ = fov_camera_deg_;
        ROS_WARN("fov angle is smaller than the fov of the camera. fov angle is set to the fov value of the camera");
    }
    double fov_angle_rad = fov_angle_deg_ * (M_PI / 180);

    // fov that can be seen by the cam
    fov_camera_rad = fov_camera_deg_ * (M_PI / 180);

    // type of cost function
    kind_cost_ = request.kind_cost;

    // get map
    if (!getMap()) {
        return false;
    }

    std::vector<bool> added_before;

    std::vector<int> all_costs = {COST_TYPE::NUMBER_OF_CELLS, COST_TYPE::BASELINE, COST_TYPE::CUSTOM_PROB,
                                  COST_TYPE::CUSTOM_PROB_WEIGHT_STATIC, COST_TYPE::CUSTOM_PROB_WEIGHT_DYNAMIC,
                                  COST_TYPE::CUSTOM_PROB_WEIGHT_DYNAMIC_AND_OBS, COST_TYPE::RANDOM,
                                  COST_TYPE::INTERPOLATION, COST_TYPE::CUSTOM_PROB_WEIGHT_STATIC_AND_OBS,
                                  COST_TYPE::CUSTOM_PROB_AND_OBS};
    std::vector<double> max_utility, max_entropy;
    max_utility.resize(all_costs.size(), 0);
    max_entropy.resize(all_costs.size(), 0);
    std::vector<int> final_location_index;
    final_location_index.resize(all_costs.size(), -1);

    std::vector<geometry_msgs::Pose> final_location;
    final_location.resize(all_costs.size());

    int wanted_kind = kind_cost_;
    int cnt = 0;
    int index = 0;

    // create a minimum bounding box
    for (auto l:request.paths.data) {
        if (!added_before.empty())
            added_before.clear();
        added_before.resize(latest_map_msg_.data.size(), false);

        std::vector<double> entropy, tot_utility;
        entropy.resize(all_costs.size(), 0);
        tot_utility.resize(all_costs.size(), 0);

        double dist = 0;

        ROS_INFO_STREAM("begin path");
        double total_weight = 0;

        // compute utility every meter
        for (int k = 0; k < l.waypoints.size(); k += 2) {

            if (k > 0) {
                dist += std::pow(std::pow(l.waypoints.at(k).position.x - l.waypoints.at(k - 1).position.x, 2) +
                                 std::pow(l.waypoints.at(k).position.y - l.waypoints.at(k - 1).position.y, 2), 0.5);
                dist += std::pow(std::pow(l.waypoints.at(k - 1).position.x - l.waypoints.at(k - 2).position.x, 2) +
                                 std::pow(l.waypoints.at(k - 1).position.y - l.waypoints.at(k - 2).position.y, 2), 0.5);
            }

            // todo if (!request.only_last )
            if (!request.only_last || k + 1 == l.waypoints.size()) {
                geometry_msgs::Point point;

                double x_start = normalizeToResolution(l.waypoints.at(k).position.x);
                double y_start = normalizeToResolution(l.waypoints.at(k).position.y);

                double theta_start_rad;
                double actual_fov;

                theta_start_rad = 0;
                actual_fov = 2 * M_PI;

                // todo override actual_fov to 360

                double theta_start_deg = theta_start_rad * (180 / M_PI);

                std::vector<std::pair<double, double>> occuVector; // angle, distance
                std::vector<std::pair<double, double>> min_max_angle_occu_vec; // min and max angles for each occupied cell
                std::vector<std::pair<double, double>> cellPos; // x,y
                std::vector<std::pair<double, double>> min_max_angle_vec;
                std::vector<std::pair<double, double>> cellVector;
                std::vector<int> prob_vec;

                if (wanted_kind == COST_TYPE::INTERPOLATION) {
                    actual_fov = 1.5*fov_camera_rad;

                    k + 2 > l.waypoints.size() - 1 ? theta_start_deg = 180 / M_PI * atan2(l.waypoints.at(k).position.y -
                                                                                          l.waypoints.at(
                                                                                                  k - 2).position.y,
                                                                                          l.waypoints.at(k).position.x -
                                                                                          l.waypoints.at(
                                                                                                  k - 2).position.x) :
                            theta_start_deg =
                                    180 / M_PI * atan2(l.waypoints.at(k + 2).position.y - l.waypoints.at(k).position.y,
                                                       l.waypoints.at(k + 2).position.x - l.waypoints.at(k).position.x);
                    theta_start_rad = theta_start_deg * M_PI / 180;
                }

                cellInFov(theta_start_rad, actual_fov, x_start, y_start,
                          occuVector, min_max_angle_occu_vec, cellPos, min_max_angle_vec, cellVector, prob_vec,
                          added_before);

                std::vector<std::pair<int, int>> visible_cells;
                std::vector<std::pair<double, double>> visibleCellsPos;
                if (!cellVector.empty())
                    raytracing(occuVector, min_max_angle_occu_vec, cellPos, min_max_angle_vec, cellVector, prob_vec,
                               visible_cells, visibleCellsPos, added_before);

                double cur_head;
                k < 2 ? cur_head = theta_start_deg : cur_head = getTheta(l.waypoints.at(k - 2).orientation);

                double wouldbechosen_angle = theta_start_deg;//tf::getYaw(l.waypoints.at(k).orientation) * 180 / M_PI;

                for (auto cost : all_costs) {
                    std::vector<int> max_utility_angles;
                    kind_cost_ = cost;
                    if (debug_ || cost == wanted_kind)
                        if (!visible_cells.empty())
                            tot_utility[cost] += getMaxUtility(visible_cells, visibleCellsPos, x_start, y_start,
                                                               max_utility_angles,
                                                               dist, l.length,
                                                               wouldbechosen_angle);
                    double min_diff = 360;
                    double chosen_angle = wouldbechosen_angle;
                    for (auto i: max_utility_angles) {
                        double angle_diff = i - cur_head;
                        angle_diff < 0 ? angle_diff += 360 : angle_diff;
                        angle_diff > 360 ? angle_diff -= 360 : angle_diff;
                        if (angle_diff <= min_diff) {
                            chosen_angle = i;
                        }
                    }
                    if (cost == wanted_kind) {
                        total_weight += std::exp(-0.25 * dist);
                        l.waypoints.at(k).orientation = tf::createQuaternionMsgFromYaw(chosen_angle * M_PI / 180);
                        if (k > 0) {
                            double diff = std::fmod(chosen_angle - cur_head + 180 + 360, 360) - 180;
                            l.waypoints.at(k - 1).orientation =
                                    tf::createQuaternionMsgFromYaw(std::fmod(cur_head + diff / 2, 360) * M_PI / 180);
                        }
                    }

                    if (debug_ || cost == wanted_kind) {
                        visualization_msgs::Marker marker;
                        marker.header.frame_id = "/map";
                        marker.header.stamp = ros::Time::now();
                        marker.ns = std::to_string(cost);
                        marker.type = visualization_msgs::Marker::ARROW;
                        marker.action = visualization_msgs::Marker::ADD;

                        marker.id = cnt;

                        marker.pose.position.x = l.waypoints.at(k).position.x;
                        marker.pose.position.y = l.waypoints.at(k).position.y;
                        marker.pose.position.z = 0;
                        marker.pose.orientation = tf::createQuaternionMsgFromYaw(chosen_angle * M_PI / 180);

                        marker.scale.x = 1;
                        marker.scale.y = 0.1;
                        marker.scale.z = 0.1;
                        if (cost == COST_TYPE::NUMBER_OF_CELLS) {
                            marker.color.r = 1.0f;
                            marker.color.g = 0.0f;
                            marker.color.b = 0.0f;
                        } else if (cost == COST_TYPE::BASELINE) {
                            marker.color.r = 0.0f;
                            marker.color.g = 1.0f;
                            marker.color.b = 0.0f;
                        } else if (cost == COST_TYPE::CUSTOM_PROB) {
                            marker.color.r = 0.0f;
                            marker.color.g = 0.0f;
                            marker.color.b = 1.0f;
                        } else if (cost == COST_TYPE::CUSTOM_PROB_WEIGHT_STATIC) {
                            marker.color.r = 1.0f;
                            marker.color.g = 1.0f;
                            marker.color.b = 0.0f;
                        } else if (cost == COST_TYPE::CUSTOM_PROB_WEIGHT_DYNAMIC) {
                            marker.color.r = 1.0f;
                            marker.color.g = 0.0f;
                            marker.color.b = 1.0f;
                        } else if (cost == COST_TYPE::CUSTOM_PROB_WEIGHT_DYNAMIC_AND_OBS) {
                            marker.color.r = 1.0f;
                            marker.color.g = 1.0f;
                            marker.color.b = 1.0f;
                        } else if (cost == COST_TYPE::RANDOM) {
                            marker.color.r = 0.0f;
                            marker.color.g = 1.0f;
                            marker.color.b = 1.0f;
                        } else if (cost == COST_TYPE::INTERPOLATION) {
                            marker.color.r = 0.0f;
                            marker.color.g = 0.0f;
                            marker.color.b = 0.0f;
                        } else if (cost == COST_TYPE::CUSTOM_PROB_WEIGHT_STATIC_AND_OBS) {
                            marker.color.r = 0.2f;
                            marker.color.g = 0.5f;
                            marker.color.b = 0.8f;
                        } else if (cost == COST_TYPE::CUSTOM_PROB_AND_OBS) {
                            marker.color.r = 0.8f;
                            marker.color.g = 0.5f;
                            marker.color.b = 0.2f;
                        }

                        marker.color.a = 1.0;
                        marker.lifetime = ros::Duration();
                        marker_pub.publish(marker);
                        cnt += 1;

                        int counter = 0;
                        for (auto i:visibleCellsPos) {
                            bool check_cell_best_heading = checkPoint(radius_camera_, i.first,
                                                                      i.second,
                                                                      x_start, y_start, fov_camera_rad,
                                                                      chosen_angle * M_PI / 180, min_distance_);

                            if (check_cell_best_heading) {
                                point.x = i.first;
                                point.y = i.second;
                                point.z = 0;
                                opt_heading_msg.cells.push_back(point);
                                std::vector<int> submap_id = positionToMapIndex(i.first, i.second, x_origin, y_origin,
                                                                                map_resolution_);
                                int id = mapIndexToId(submap_id.at(0), submap_id.at(1), map_width_);
                                double prob = int(latest_map_msg_.data.at(id));
                                if (cost == wanted_kind)
                                    added_before.at(id) = true;

                                prob == -1 ? prob = 50 : prob;
                                prob == 0 ? prob = 0.00000000001 : (prob == 100 ? prob = 0.999999999 : prob = prob /
                                                                                                              100.0);
                                entropy[cost] +=
                                        -(prob * std::log2(prob) + (1 - prob) * std::log2(1 - prob)) *
                                        std::exp(-0.25 * (dist));
                                counter += 1;
                            }
                        }
//                    entropy /= counter;
                        // Publish GridCells msg of optimal heading
                        opt_heading_pub_.publish(opt_heading_msg);
                    }
                }
            }
            clear_messages();
        }


        ROS_INFO_STREAM("------------------------------------------------------------------");
        ROS_INFO_STREAM("Toward position " << l.waypoints.at(l.waypoints.size() - 1).position);

        for (auto kind_considered : all_costs) {
            if (debug_ || kind_considered == wanted_kind) {
                ROS_INFO_STREAM("FOR kind of cost " << kind_considered << " we have: ");
                if (weighted_avg){
                    tot_utility[kind_considered]/= total_weight;
                    entropy[kind_considered] /= total_weight;
                }
                ROS_INFO_STREAM("We get " << tot_utility[kind_considered] << " utility");
                ROS_INFO_STREAM("We get " << entropy[kind_considered] << " entropy \n");
                if (tot_utility[kind_considered] > max_utility[kind_considered]) {
                    max_utility[kind_considered] = tot_utility[kind_considered];
                    max_entropy[kind_considered] = entropy[kind_considered];
                    final_location.at(kind_considered) = l.waypoints.at(l.waypoints.size() - 1);
                    final_location_index.at(kind_considered) = index;

                    if (kind_considered == wanted_kind) {
                        response.path.waypoints = l.waypoints;
                        response.path.length = l.length;
                    }
                }
            }
        }

        ROS_INFO_STREAM("end path");
        index++;
    }
    ROS_INFO_STREAM("------------------------------------------------------------------");
    for (auto kind_considered: all_costs) {
        if (debug_ || kind_considered == wanted_kind) {
            ROS_INFO_STREAM("FOR kind of cost " << kind_considered << " we have: ");
            ROS_INFO_STREAM("We get " << max_utility[kind_considered] << " utility");
            ROS_INFO_STREAM("We get " << max_entropy[kind_considered] << " entropy");
            ROS_INFO_STREAM("Toward " << final_location_index[kind_considered] << "\n");
        }
    }

    return true;
}

void MapExtract::clear_messages() {
    opt_heading_msg.cells.clear();
    occ_points_msg.cells.clear();
    free_points_msg.cells.clear();
    raytracing_free_msg.cells.clear();
    raytracing_occu_msg.cells.clear();
    raytracing_target_msg.cells.clear();
    raytracing_unk_msg.cells.clear();
    unk_points_msg.cells.clear();
}

void MapExtract::prepare_messages(double map_resolution_) {
    // msg for visualize occ - free - unkown cells
    occ_points_msg.header.frame_id = "map";
    occ_points_msg.cell_width = map_resolution_;
    occ_points_msg.cell_height = map_resolution_;

    free_points_msg.header.frame_id = "map";
    free_points_msg.cell_width = map_resolution_;
    free_points_msg.cell_height = map_resolution_;

    unk_points_msg.header.frame_id = "map";
    unk_points_msg.cell_width = map_resolution_;
    unk_points_msg.cell_height = map_resolution_;

    raytracing_target_msg.header.frame_id = "map";
    raytracing_target_msg.cell_width = map_resolution_;
    raytracing_target_msg.cell_height = map_resolution_;

    raytracing_free_msg.header.frame_id = "map";
    raytracing_free_msg.cell_width = map_resolution_;
    raytracing_free_msg.cell_height = map_resolution_;

    raytracing_occu_msg.header.frame_id = "map";
    raytracing_occu_msg.cell_width = map_resolution_;
    raytracing_occu_msg.cell_height = map_resolution_;

    raytracing_unk_msg.header.frame_id = "map";
    raytracing_unk_msg.cell_width = map_resolution_;
    raytracing_unk_msg.cell_height = map_resolution_;

    opt_heading_msg.header.frame_id = "map";
    opt_heading_msg.cell_width = map_resolution_;
    opt_heading_msg.cell_height = map_resolution_;
}

void MapExtract::publish_bb(double x_max, double x_min, double y_max, double y_min, double x_origin, double y_origin,
                            double map_resolution_) {
    geometry_msgs::Point point;
    nav_msgs::GridCells submap_points_bounding_msg;
    submap_points_bounding_msg.header.frame_id = "map";
    submap_points_bounding_msg.cell_width = map_resolution_;
    submap_points_bounding_msg.cell_height = map_resolution_;
    point.x = x_min;
    point.y = y_max;
    submap_points_bounding_msg.cells.push_back(point);
    point.x = x_max;
    point.y = y_max;
    submap_points_bounding_msg.cells.push_back(point);
    point.x = x_min;
    point.y = y_min;
    submap_points_bounding_msg.cells.push_back(point);
    point.x = x_max;
    point.y = y_min;
    submap_points_bounding_msg.cells.push_back(point);
    // Publish GridCells msg of frontier centroids
    submap_bounding_box_cell_pub_.publish(submap_points_bounding_msg);

    nav_msgs::GridCells origin_fov_msg;
    origin_fov_msg.header.frame_id = "map";
    origin_fov_msg.cell_width = map_resolution_;
    origin_fov_msg.cell_height = map_resolution_;
    point.x = x_origin;
    point.y = y_origin;
    origin_fov_msg.cells.push_back(point);
    origin_fov_pub_.publish(origin_fov_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_extract");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    MapExtract map_reader(nh, private_nh);

    ros::spin();

    return 0;
}