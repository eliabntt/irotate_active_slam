cmake_minimum_required(VERSION 2.8.3)
project(robotino_simulations)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        tf
        std_msgs
        sensor_msgs
        geometry_msgs
        gazebo_msgs
        nav_msgs
        cmake_modules
        laser_geometry)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_compile_options(-std=c++11)

# let cmake find OpenMP and set some variables
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message(STATUS "OPENMP FOUND")
    set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
    set(OpenMP_LIBRARIES gomp)
endif()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS roscpp roslib tf std_msgs sensor_msgs geometry_msgs nav_msgs gazebo_ros pcl_ros nodelet
        DEPENDS EIGEN3 OpenMP)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(plot_SDF src/plot_SDF.cpp)
add_executable(get_SDF src/get_SDF.cpp)
add_executable(calculate_SDF_error src/calculate_SDF_error.cpp)
add_executable(calculate_map_error src/calculate_map_error.cpp)
add_executable(pub_map_difference src/pub_map_difference.cpp)
add_executable(plot_SDF_diff src/plot_SDF_diff.cpp)
add_executable(convert_map src/convert_map.cpp)

target_compile_options(get_SDF PRIVATE ${OpenMP_FLAGS})
target_link_libraries(get_SDF ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenMP_LIBRARIES})
target_link_libraries(calculate_SDF_error ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(calculate_map_error ${catkin_LIBRARIES})
target_link_libraries(pub_map_difference ${catkin_LIBRARIES})
target_link_libraries(plot_SDF_diff ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(plot_SDF ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(convert_map ${catkin_LIBRARIES})