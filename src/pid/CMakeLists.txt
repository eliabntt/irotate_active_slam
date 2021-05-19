cmake_minimum_required(VERSION 2.8.3)
project(pid)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
)
find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
    cfg/Pid.cfg
)

catkin_package(
  INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs message_runtime dynamic_reconfigure
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(controller src/controller.cpp src/pid.cpp)
add_executable(plant_sim src/plant_sim.cpp)
add_executable(setpoint_node src/setpoint_node.cpp)
add_executable(sim_time src/sim_time.cpp)
add_executable(autotune src/autotune.cpp)

add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(plant_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(setpoint_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sim_time ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(autotune ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(controller ${catkin_LIBRARIES})
target_link_libraries(plant_sim ${catkin_LIBRARIES})
target_link_libraries(setpoint_node ${catkin_LIBRARIES})
target_link_libraries(sim_time ${catkin_LIBRARIES})
target_link_libraries(autotune ${catkin_LIBRARIES})

install(TARGETS controller plant_sim setpoint_node sim_time autotune
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
