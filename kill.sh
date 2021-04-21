
#!/bin/bash
sleep 303
rosservice call /rtabmap/pause
pkill -f "robotino_mpc"
pkill -f "rtabmap_eval"
