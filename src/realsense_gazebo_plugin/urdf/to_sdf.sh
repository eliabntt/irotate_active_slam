#!/bin/bash

# Go to realsense_gazebo_plugin/urdf folder
roscd realsense_gazebo_plugin/urdf

# Convert xacro to urdf
rosrun xacro xacro --inorder d435.xacro -o d435.urdf

# Convert urdf to sdf
gz sdf -p d435.urdf > d435.sdf

copy sdf file to model folder
cp d435.sdf ../model/
