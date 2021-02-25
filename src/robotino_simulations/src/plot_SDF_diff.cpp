#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


int main(int argc, char **argv)
{
    // Get path and file name
    std::string package_path = ros::package::getPath("robotino_simulations");
    std::string sdf_path = "/home/ebonetto/Desktop/Test/E1/sdf.txt";
    std::string sdf_true_path ="/home/ebonetto/Desktop/Test/E1/gt_sdf.txt";

    std::ifstream sdf_file(sdf_path.c_str());
    std::ifstream sdf_true_file(sdf_true_path.c_str());

    std::vector<std::vector<float>> pre_mat;
    std::string line;
    if (sdf_file.is_open()){
        while (std::getline(sdf_file, line)){
            std::stringstream ss;
            ss << line;
            float dist;
            std::vector<float> temp_vec;
            while (ss >> dist){
                temp_vec.push_back(dist);
            }
            pre_mat.push_back(temp_vec);
        }
    }
    else {
        ROS_INFO("Failed to open SDF file!");
    }

    int width = pre_mat.size();
    int height = pre_mat[0].size();
    Eigen::MatrixXf sdf_mat(width, height);
    for (int i = 0; i < width; i++){
        for (int j = 0; j < height; j++){
            sdf_mat(i, j) = pre_mat[i][j];
        }
    }

    std::vector<std::vector<float>> pre_true_mat;
    if (sdf_true_file.is_open()){
        while (std::getline(sdf_true_file, line)){
            std::stringstream ss;
            ss << line;
            float dist;
            std::vector<float> temp_vec;
            while (ss >> dist){
                temp_vec.push_back(dist);
            }
            pre_true_mat.push_back(temp_vec);
        }
    }
    else {
        ROS_INFO("Failed to open SDF file!");
    }

    Eigen::MatrixXf sdf_true_mat(width, height);
    for (int i = 0; i < width; i++){
        for (int j = 0; j < height; j++){
            sdf_true_mat(i, j) = pre_true_mat[i][j];
        }
    }

    Eigen::MatrixXf sdf_diff(width, height);
    sdf_diff = sdf_true_mat - sdf_mat;
    sdf_diff = sdf_diff.cwiseAbs();

    double max_error = 0;
    double min_error = 1000;
    for (int i = 0; i < width; i++){
        for (int j = 0; j < height; j++){
            // if (sdf_true_mat(i, j) < 0) sdf_diff(i, j) = 0;
            // if (sdf_diff(i, j) <= 1) sdf_diff(i, j) = 0;
            // else sdf_diff(i, j) -= 1;
            if (sdf_diff(i, j) > max_error) max_error = sdf_diff(i, j);
            if (sdf_diff(i, j) < min_error) min_error = sdf_diff(i, j);
        }
    }

    std::cout << "max_error: " << max_error << std::endl;
    std::cout << "min_error: " << min_error << std::endl;

    // Show SDF as image
    cv::Mat sdf_cv(width, height, CV_64F);
    cv::eigen2cv(sdf_diff, sdf_cv);

    cv::Mat image;
    cv::normalize(sdf_cv, image, 0, 1, cv::NORM_MINMAX);

    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display window", image);
    cv::waitKey(0);

    return 0;
}