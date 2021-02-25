cmake_minimum_required(VERSION 2.8.3)
project(active_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
nav_msgs
std_msgs
geometry_msgs
roscpp
rospy
grid_map_ros
message_generation)

add_service_files(
  FILES
  get_frontier_list.srv
  get_best_path.srv
  get_best_head.srv
)

add_message_files(
        FILES
        AvailablePaths.msg
        SinglePath.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
)

add_compile_options(-std=c++11)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
CATKIN_DEPENDS
nav_msgs
std_msgs
geometry_msgs
roscpp
rospy
message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

# Executables
add_executable(map_extract_node src/map_extract.cpp)
add_executable(frontier_exploration_node src/frontier_exploration.cpp)
add_executable(frontier_backup src/frontier_backup.cpp)

# Dependencies
add_dependencies(map_extract_node active_slam_generate_messages_cpp)
add_dependencies(frontier_exploration_node active_slam_generate_messages_cpp)
add_dependencies(frontier_backup active_slam_generate_messages_cpp)

# Link libraries
target_link_libraries(map_extract_node ${catkin_LIBRARIES})
target_link_libraries(frontier_exploration_node ${catkin_LIBRARIES})
target_link_libraries(frontier_backup ${catkin_LIBRARIES})




