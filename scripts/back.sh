#/bin/bash
IFS='/'
str="$*"
ls /home/ebonetto/Desktop/Test/"$str"
./map.sh "$str"
python evaluate_ate.py rtabmap_slam.txt rtabmap_gt.txt --plot ate.pdf --verbose > ate.txt
python evaluate_rpe.py rtabmap_odom.txt rtabmap_gt.txt --plot rpe.pdf --fixed_delta  --verbose > rpe.txt
mv -i *.txt /home/ebonetto/Desktop/Test/"$str"
mv -i *.npy /home/ebonetto/Desktop/Test/"$str"
mv -i rtabmap.db /home/ebonetto/Desktop/Test/"$str"
