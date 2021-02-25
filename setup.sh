#!bin/bash
mkdir -p ./devel/include/robotino_msgs
cp ./../tmp_robotino_exp/robotino_msgs/*.h ./devel/include/robotino_msgs
mkdir ~/.gazebo/models/kitchen_dining/
rm ~/.gazebo/models/kitchen_dining/* -r
cp ./kitchen_dining/* ~/.gazebo/models/kitchen_dining/ -r
catkin_make
