#/bin/bash
sleep 1
source ./devel/setup.bash
#catkin_make
rosrun robotino_simulations convert_map
#rosrun robotino_simulations get_SDF
#rosrun robotino_simulations plot_SDF
rosrun robotino_simulations calculate_map_error
#rosrun robotino_simulations calculate_SDF_error
#rosrun robotino_simulations pub_map_difference
#rosrun robotino_simulations plot_SDF_diff
IFS='/'
str="$*"
rosrun map_server map_saver -f map
mv map.yaml /home/ebonetto/Desktop/Test/"$str"/
mv map.pgm /home/ebonetto/Desktop/Test/"$str"/
mv /home/ebonetto/Desktop/Test/E1/occupancy.txt /home/ebonetto/Desktop/Test/"$str"
#mv /home/ebonetto/Desktop/Test/E1/sdf.txt /home/ebonetto/Desktop/Test/"$str"
mv /home/ebonetto/Desktop/Test/E1/general_results.txt /home/ebonetto/Desktop/Test/"$str"
#mv /home/ebonetto/Desktop/Test/E1/general_results_SDF.txt /home/ebonetto/Desktop/Test/"$str"

cp ~/.ros/rtabmap.db rtabmap.db && rtabmap-report --poses ./rtabmap.db
