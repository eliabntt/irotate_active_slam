cmake_minimum_required(VERSION 2.8.3)
project(robotino_camera_heading)

add_compile_options("-std=c++17")

find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        std_msgs
        geometry_msgs
        roscpp
        octomap_msgs
        message_generation
        rtabmap_ros)

find_package(OpenCV)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
        octomap_msgs
)

catkin_package(
        CATKIN_DEPENDS
        nav_msgs
        std_msgs
        geometry_msgs
        roscpp
        message_runtime
        octomap_msgs
        rtabmap_ros
        LIBRARIES rtabmap_ros
	DEPENDS OpenCV
)

SET(Libraries
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    )

include_directories(${OpenCV_INCLUDE_DIRS}
        include
        ${catkin_INCLUDE_DIRS})

add_executable(camera_best_heading_node src/camera_best_heading.cpp)

target_link_libraries(camera_best_heading_node ${catkin_LIBRARIES})
