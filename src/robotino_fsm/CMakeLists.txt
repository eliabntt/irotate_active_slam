cmake_minimum_required(VERSION 2.8.3)
project(robotino_fsm)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        std_msgs
        geometry_msgs
        roscpp
        tf
        rtabmap_ros
        message_filters
	active_slam
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        nav_msgs
        std_msgs
        geometry_msgs
        roscpp
        tf
        message_filters
        rtabmap_ros
	active_slam
        LIBRARIES rtabmap_ros
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

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/robotino_fsm_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
