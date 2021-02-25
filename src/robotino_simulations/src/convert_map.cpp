#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

ros::ServiceClient get_map_client_;

void saveGridMap(){
  // Get path and file name
  std::string package_path = ros::package::getPath("robotino_simulations");
  std::string save_path = "/home/ebonetto/Desktop/Test/E1/occupancy.txt";

  // Service call for latest occupancy grid map
  nav_msgs::GetMap get_map_srv;

  if (get_map_client_.call(get_map_srv))
  {
    ROS_INFO("OccupancyGrid map call successfull");
  }
  else
  {
    ROS_ERROR("Failed to call OccupancyGrid map");
  }

  nav_msgs::OccupancyGrid latest_map_msg_ = get_map_srv.response.map;

  // Save map
  std::ofstream map_file(save_path.c_str());
  if (map_file.is_open()){
    for (int i = 0; i < latest_map_msg_.info.width * latest_map_msg_.info.height; i++){
      map_file << int(latest_map_msg_.data[i]) << std::endl;
    }
    map_file.close();
  }
  else{
    ROS_INFO("Could not open occupancy_grid_map.txt!");
  }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_node_name");

    ros::NodeHandle nh_;
    get_map_client_ = nh_.serviceClient<nav_msgs::GetMap>
            ("/rtabmap/get_map");
    saveGridMap();
    return 0;
}