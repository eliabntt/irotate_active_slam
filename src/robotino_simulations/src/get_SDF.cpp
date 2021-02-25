#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>


void mapIdToIndex(int id, int& ix_cell, int& iy_cell, int map_width){
    iy_cell = id / map_width;
    ix_cell = id - iy_cell * map_width;
}

std::vector<int> positionToMapIndex(double x, double y,
                                    unsigned int width, unsigned int height, float resolution){
    std::vector<int> index(2);
    index[0] = floor(x / resolution) + width / 2;
    index[1] = floor(y / resolution) + height / 2;

    return index;
}

void findChange(bool& change, int occupation, int current_occupation){
    if (current_occupation != occupation){
        if (std::abs(current_occupation + occupation) == 1){
            change = true;
        }
    }
}

double xCircleMinDist(int ix, int iy, int n_cells, Eigen::MatrixXi& map){
    // https://www.geeksforgeeks.org/bresenhams-circle-drawing-algorithm/
    float dist_sq = -1;
    int occupation = map(ix, iy);
    int x = n_cells;
    int y = 0;
    int err = 3 - (n_cells << 1);
    bool found_change = false;

    int xc, yc;
    if (occupation != 0){
        xc = x - 1;
        yc = y;
    }
    else {
        xc = x;
        yc = y;
    }

    while (x >= y){
            findChange(found_change, occupation, map(ix + x, iy + y)); // 1. octant
            findChange(found_change, occupation, map(ix + x, iy - y)); // 8. octant
            findChange(found_change, occupation, map(ix - y, iy + x)); // 3. octant
            findChange(found_change, occupation, map(ix - x, iy + y)); // 4. octant
            findChange(found_change, occupation, map(ix - x, iy - y)); // 5. octant
            findChange(found_change, occupation, map(ix - y, iy - x)); // 6. octant
            findChange(found_change, occupation, map(ix + y, iy + x)); // 2. octant
            findChange(found_change, occupation, map(ix + y, iy - x)); // 7. octant

        if (found_change){
            double dist_sq_temp = xc * xc + yc * yc;
            if (dist_sq == -1 || dist_sq > dist_sq_temp){
                dist_sq = dist_sq_temp;
            }
        }

        y++;
        if (err > 0){
            x--;
            err += ((y - x) << 2) + 10; //<< 2 -> 4*
        }
        else {
            err += (y << 2) + 6;
        }

        if (occupation != 0){
            yc = y - 1;
            xc = x - 1;
        }
        else {
            yc = y;
            xc = x;
        }
        found_change = false;
    }
    if (dist_sq != -1) return sqrt(dist_sq);
    else return dist_sq;
}

int main(int argc, char **argv)
{
    // Init params
    int map_width = 621;
    int map_height = 621;
    float map_resolution = 0.05;
    Eigen::MatrixXi occ_mat(map_width, map_height);

    // Get path and file name
    std::string package_path = ros::package::getPath("robotino_simulations");
    std::string save_directory_path = "/home/ebonetto/Desktop/Test/E1/";
    std::string map_path = "/home/ebonetto/Desktop/Test/E1/occupancy.txt";
    std::string save_path = "/home/ebonetto/Desktop/Test/E1/sdf.txt";;

    std::ifstream map_file(map_path.c_str());

    std::string line;
    int id = 0;

    if (map_file.is_open()){
        int x_cell, y_cell;
        while (std::getline(map_file, line)){
            mapIdToIndex(id, x_cell, y_cell, map_width);
            std::stringstream ss_ref;
            ss_ref << line;
            double p_ref;
            ss_ref >> p_ref;

            if (p_ref == -1){
                occ_mat(x_cell, y_cell) = -1;
            }
            else if (p_ref >= 50){
                occ_mat(x_cell, y_cell) = 1;
            }
            else {
                occ_mat(x_cell, y_cell) = 0;
            }
            id += 1;
        }
    }
    else {
        ROS_INFO("Failed to open map_file!");
    }

    // SDF calculation
    // World borders (inflated by 1m from wall) for utm_0 world
//    double p1[2] = {-6, -12};
//    double p2[2] = {5, 12};
// E1
    double p1[2] = {-12, -5};
    double p2[2] = {12, 6};

    std::vector<int> p_start = positionToMapIndex(p1[0], p1[1], map_width, map_height, map_resolution);
    ROS_INFO_STREAM(p_start[0]);
    ROS_INFO_STREAM(p_start[1]);
    std::vector<int> p_end = positionToMapIndex(p2[0], p2[1], map_width, map_height, map_resolution);
    ROS_INFO_STREAM(p_end[0]);
    ROS_INFO_STREAM(p_end[1]);

    int sdf_width = p_end[0] - p_start[0] + 2;
    int sdf_height = p_end[1] - p_start[1] + 2;
    Eigen::MatrixXd SDF_mat(sdf_width, sdf_height);

    int border = 0;
    int counter = 0;
    // Calculate SDF
#pragma omp parallel for
    for (int i = p_start[0]; i <= p_end[0]; i++){
        int n = 1;
        double dist = -1;
        bool not_found = true;
        for (int j = p_start[1]; j <= p_end[1]; j++){
            // Search for shortest distance of (i, j) and assign distance to SDF map
            dist = -1;
            not_found = true;
            while (not_found){
                dist = xCircleMinDist(i, j, n, occ_mat);
                if (dist == -1){
                    n += 1;
                }
                else {
                    not_found = false;
                    double temp_dist = xCircleMinDist(i, j, n + 1, occ_mat);
                    if (dist > temp_dist && temp_dist != -1){
                        dist = temp_dist;
                    }
                    if (n > 2) n -= 2;
                    else n = 1;
                }
            }

            if (occ_mat(i, j) == 0){
                SDF_mat(i - p_start[0], j - p_start[1]) = dist;
            }
            else {
                SDF_mat(i - p_start[0], j - p_start[1]) = -dist;
            }
        }

#pragma omp atomic
        counter++;
        int percent = 100 * counter / sdf_width;
        if (percent >= border){
            std::cout << percent << " %" << std::endl;
#pragma omp atomic
            border += 1;
        }
    }

    // Save map
    std::ofstream sdf_file(save_path.c_str());
    if (sdf_file.is_open()){
        sdf_file << SDF_mat;
        sdf_file.close();
    }
    else{
        ROS_INFO("Could not save sdf_map.txt!");
    }
}