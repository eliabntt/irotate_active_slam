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

    std::ifstream sdf_file(sdf_path.c_str());

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

    // Show SDF as image
    cv::Mat sdf_cv(width, height, CV_64F);
    cv::eigen2cv(sdf_mat, sdf_cv);

    cv::Mat image;
    cv::normalize(sdf_cv, image, 0, 1, cv::NORM_MINMAX);

    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display window", image);
    cv::waitKey(0);

    return 0;
}
