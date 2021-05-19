
#!/bin/bash
sleep 303
rosservice call /rtabmap/pause
pkill -f "robotino_mpc"
pkill -f "robotino_fsm"
pkill -f "move_base"
pkill -f "rtabmap_eval"
