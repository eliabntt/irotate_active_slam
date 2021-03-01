#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>


int main(int argc, char **argv)
{
    // Get path and file name
    std::string package_path = ros::package::getPath("robotino_simulations");
    std::string directory_path =   "/home/ebonetto/Desktop/Test/E1";

    std::string general_results_path = directory_path + "/general_results.txt";
    std::string test_results_path = directory_path + "/occupancy.txt";
    std::string ground_truth_map_path = directory_path + "/gt_occupancy.txt";

    std::ifstream test_map_file(test_results_path.c_str());
    std::ifstream ground_truth_map_file(ground_truth_map_path.c_str());

    std::string line;
    int i = 0;
    int j = 0;
    int k = 0;
    double map_score = 0;
    double over_ref_score = 0;
    double correct_free = 0;
    double total_free = 0;
    double correct_occ = 0;
    double total_occ = 0;
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


                if (p_test == 100 || p_ref == 100)
                {
                    total_occ ++;
                }
                if (p_test == 0 || p_ref == 0){
                    total_free ++;
                }
                if (p_test != -1){
                    if (p_test == 100){
                        if (p_ref == p_test){
                            correct_occ++;
                        }
                    }
                    else{
                        if (p_ref == p_test){
                            correct_free++;
                        }
                    }
                }

                // Map Scoring
                if (p_test == -1){
                    p_test = 0.5;
                    if (p_ref == -1){
                        p_ref = 0.5;
                    }
                    else {
                        k += 1;
                        p_ref /= 100;
                    }
                }
                else {
                    p_test /= 100;
                    i += 1;
                    if (p_ref == -1){
                        p_ref = 0.5;
                        j += 1;
                        over_ref_score += std::pow(p_ref - p_test, 2);
                    }
                    else {
                        p_ref /= 100;
                    }
                }
                map_score += std::pow(p_ref - p_test, 2);
            }
        }
        else {
            ROS_INFO("Failed to open test_map_file!");
        }
    }
    else {
        ROS_INFO("Failed to open ground_truth_map_file!");
    }

    std::ofstream result_file(general_results_path.c_str(), std::ofstream::app);
    if (result_file.is_open()){
        // Add information to file
        result_file << std::endl;
        result_file << "Map:" << std::endl;
        result_file << "map_score: " << map_score << std::endl;
        result_file << "Number of known cells: " << i << std::endl;
        result_file << "Number of known test cells, while unknown in ref map: " << j << std::endl;
        result_file << "score for known test cells, while unkown in ref map: " << over_ref_score << std::endl;
        result_file << "Number of known ref cells, while unknown in test map: " << k << std::endl;
//        result_file << "BAC score " << 1/2.0*(correct_free/total_free + correct_occ/total_occ) << std::endl;
//        result_file << "BAC correct_free_ratio " << (correct_free/total_free) << std::endl;
//        result_file << "BAC correct_occ_ratio " << (correct_occ/total_occ) << std::endl;

        // Close file
        result_file.close();
    }
    else{
        ROS_INFO("Could not open general_results.txt!");
    }

    std::cout << "map_score: " << map_score << std::endl;
    std::cout << "Number of known cells: " << i << std::endl;
    std::cout << "Number of known test cells, while unknown in ref map: " << j << std::endl;
    std::cout << "score for known test cells, while unkown in ref map: " << over_ref_score << std::endl;
    std::cout << "Number of known ref cells, while unknown in test map: " << k << std::endl;
    return 0;
}