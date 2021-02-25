cmake_minimum_required(VERSION 2.8.3)
project(robotino_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        std_msgs
        geometry_msgs
        roscpp
        rospy
        costmap_converter
        dynamic_reconfigure
        tf
        sensor_msgs
        message_filters
        robotino_fsm
        )

add_compile_options(-std=c++11)

generate_dynamic_reconfigure_options(
        cfg/NonLinearMPC.cfg
)
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include cfg
        CATKIN_DEPENDS
        nav_msgs
        std_msgs
        geometry_msgs
        roscpp
        rospy
        costmap_converter
        robotino_fsm
)


add_library(robotino_mpc_lib
        src/nlmpc.cpp
        solver_made_from_cpp/qpoases/SRC/Bounds.cpp
        solver_made_from_cpp/qpoases/SRC/Constraints.cpp
        solver_made_from_cpp/qpoases/SRC/CyclingManager.cpp
        solver_made_from_cpp/qpoases/SRC/Indexlist.cpp
        solver_made_from_cpp/qpoases/SRC/MessageHandling.cpp
        solver_made_from_cpp/qpoases/SRC/QProblem.cpp
        solver_made_from_cpp/qpoases/SRC/QProblemB.cpp
        solver_made_from_cpp/qpoases/SRC/SubjectTo.cpp
        solver_made_from_cpp/qpoases/SRC/Utils.cpp
        solver_made_from_cpp/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
        solver_made_from_cpp/solver/acado_qpoases_interface.cpp
        solver_made_from_cpp/solver/acado_integrator.c
        solver_made_from_cpp/solver/acado_solver.c
        solver_made_from_cpp/solver/acado_auxiliary_functions.c
        )

include_directories(
        include
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}/robotino_mpc
        solver_made_from_cpp/solver
        solver_made_from_cpp/qpoases
        solver_made_from_cpp/qpoases/INCLUDE
        solver_made_from_cpp/qpoases/SRC
        )


###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
install()

# Executables
add_executable(robotino_mpc src/nlmpc_node.cpp)

# Dependencies
add_dependencies(robotino_mpc ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(robotino_mpc_lib ${catkin_LIBRARIES})
# Link libraries
target_link_libraries(robotino_mpc ${catkin_LIBRARIES} robotino_mpc_lib)





