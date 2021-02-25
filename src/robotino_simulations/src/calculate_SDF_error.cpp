#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>

Eigen::MatrixXf load_SDF(std::string sdf_path){
    // Open file
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
        ROS_INFO("failed");
    }

    int width = pre_mat.size();
    int height = pre_mat[0].size();
    Eigen::MatrixXf sdf_mat(width, height);
    for (int i = 0; i < width; i++){
        for (int j = 0; j < height; j++){
            sdf_mat(i, j) = pre_mat[i][j];
        }
    }
    return sdf_mat;
}


int main(int argc, char **argv)
{
    // Get path and file name
    std::string package_path = ros::package::getPath("robotino_simulations");
    std::string directory_path = "/home/ebonetto/Desktop/Test/E1";

    std::string general_results_path = directory_path + "/general_results_SDF.txt";
    std::string test_sdf_path = directory_path + "/sdf.txt";
    std::string ref_sdf_path = directory_path + "/gt_sdf.txt";

    Eigen::MatrixXf test_sdf_mat = load_SDF(test_sdf_path);
    Eigen::MatrixXf ref_sdf_mat = load_SDF(ref_sdf_path);

    // Calculate score
    double sdf_score_full = 0;
    double sdf_score_easy = 0;

    for (int i = 0; i < test_sdf_mat.rows(); i++){
        for (int j = 0; j < test_sdf_mat.cols(); j++){
            sdf_score_full += pow(std::abs(test_sdf_mat(i, j) - ref_sdf_mat(i, j)), 2);
            if (ref_sdf_mat(i, j) >= 0){
                if (std::abs(test_sdf_mat(i, j) - ref_sdf_mat(i, j)) > 1){
                    double score_increment = std::abs(test_sdf_mat(i, j) - ref_sdf_mat(i, j)) - 1;
                    sdf_score_easy += pow(score_increment, 2);
                }
            }
        }
    }

    std::ofstream result_file(general_results_path.c_str(), std::ofstream::app);
    if (result_file.is_open()){
        // Add information to file
        result_file << std::endl;
        result_file << "sdf_score_full: " << sdf_score_full << std::endl;
        result_file << "sdf_score_easy: " << sdf_score_easy << std::endl;

        // Close file
        result_file.close();
    }
    else{
        ROS_INFO("Could not open general_results.txt!");
    }

    std::cout << "sdf_score_full: " << sdf_score_full << std::endl;
    std::cout << "sdf_score_easy: " << sdf_score_easy << std::endl;
    return 0;
}