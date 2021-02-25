#!bin/bash

cd ~/active-rgbd-slam/ros_slam/src/robotino_simulations/models
mkdir 3dgems_temp

cd ~/active-rgbd-slam/ros_slam/src/robotino_simulations/models/3dgems_temp

wget -O decoration.tar.gz data.nvision2.eecs.yorku.ca/3DGEMS/data/decoration.tar.gz && tar -xvf decoration.tar.gz
wget -O earthquake.tar.gz data.nvision2.eecs.yorku.ca/3DGEMS/data/earthquake.tar.gz && tar -xvf earthquake.tar.gz
wget -O electronics.tar.gz data.nvision2.eecs.yorku.ca/3DGEMS/data/electronics.tar.gz && tar -xvf electronics.tar.gz
wget -O food.tar.gz data.nvision2.eecs.yorku.ca/3DGEMS/data/food.tar.gz && tar -xvf food.tar.gz
wget -O furniture.tar.gz data.nvision2.eecs.yorku.ca/3DGEMS/data/furniture.tar.gz && tar -xvf furniture.tar.gz
wget -O kitchen.tar.gz data.nvision2.eecs.yorku.ca/3DGEMS/data/kitchen.tar.gz && tar -xvf kitchen.tar.gz
wget -O miscellaneous.tar.gz data.nvision2.eecs.yorku.ca/3DGEMS/data/miscellaneous.tar.gz && tar -xvf miscellaneous.tar.gz
wget -O shapes.tar.gz data.nvision2.eecs.yorku.ca/3DGEMS/data/shapes.tar.gz && tar -xvf shapes.tar.gz
wget -O stationery.tar.gz data.nvision2.eecs.yorku.ca/3DGEMS/data/stationery.tar.gz && tar -xvf stationery.tar.gz
wget -O tools.tar.gz data.nvision2.eecs.yorku.ca/3DGEMS/data/tools.tar.gz && tar -xvf tools.tar.gz

rm decoration.tar.gz
rm earthquake.tar.gz
rm electronics.tar.gz
rm food.tar.gz
rm furniture.tar.gz
rm kitchen.tar.gz
rm miscellaneous.tar.gz
rm shapes.tar.gz
rm stationery.tar.gz
rm tools.tar.gz

mv decoration/* .
mv earthquake/* .
mv electronics/* .
mv food/* .
mv furniture/* .
mv kitchen/* .
mv miscellaneous/* .
mv shapes/* .
mv stationery/* .
mv tools/* .

rm -r decoration
rm -r earthquake
rm -r electronics
rm -r food
rm -r furniture
rm -r kitchen
rm -r miscellaneous
rm -r shapes
rm -r stationery
rm -r tools


cd ~/active-rgbd-slam/ros_slam/src/robotino_simulations/models

mv ~/active-rgbd-slam/ros_slam/src/robotino_simulations/models/3dgems_temp ~/.gazebo/models/

#cp ./kitchen_dining ~/.gazebo/models/kitchen_dining/ -r







