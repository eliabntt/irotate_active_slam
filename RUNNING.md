Since you will need at least 5 terminals to run this project my suggestion is to use a package like _terminator_.
1. Launch the simulation environment `roslaunch robotino_simulations world.launch` [here](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_simulations/launch/world.launch). 
    1. Select the desired environment with `name:=<arg>`. We tested our work with `name:="cafe"` and `name:="small_house" x:=-2.0`.
       
       .world files are placed in `robotino_simulations/worlds/`
    2. This node takes care also of launching the nodes for generating the necessary ground truth transforms
    3. In case a shift is needed (like with the AWS's small_house) the `robot_ekf` in `robotino_description/launch/robotino.launch` should be adapted to get the corrispondent starting pose so that groundtruth can reflect this shift
2. Run the [MPC](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_mpc/launch/robotino_mpc.launch):  `roslaunch robotino_mpc robotino_mpc.launch`. [Parameters](https://github.com/eliabntt/active_v_slam/tree/master/src/robotino_mpc/cfg) in `robotino_mpc/cfg/nmpc.yaml` will be used in the code to configure mainly the topics while NMPC constraints can be configured either dynamically (`rosrun rqt_reconfigure rqt_reconfigure`) or by editing the `robotino_mpc/cfg/NonLinearMPC.cfg` file. Note that the latter require a `catkin_make`.
3. Run RTABMap: `roslaunch robotino_simulations rtabmap.launch` which is [here](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_simulations/launch/rtabmap.launch). Many things will be launched here.
    1. `move_base` and the costmap converter found [here](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_simulations/launch/move_base.launch). The parameters are placed in `robotino_simulations/config`
    2. The \"main\" [`mapping.launch`](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_simulations/launch/mapping.launch) file contains:
        - `libpointmatcher` odometry node
        - `rgbd_sync` node (from RTABMap)
        - config parameters of RTABMap
    3. the argument `delete:=-d` lets the user delete the previously generated db at launch
    4. the default folder of the db is `.ros/rtabmap.db` (setted with the `db` argument)
    5. RTABMap parameters can be edited directly in `robotino_simulations/launch/mapping.launch`
    6. RTABMapViz (the visualization tool of RTABMap) is currently commented in `robotino_simulations/launch/mapping.launch`. Note that with this enabled performance will suffer most probably requiring RTABMap to use its memory management techniques.
    7. Note that some arguments, like "compressed" (which is triggerred by remote:=true) are useful only in case of distributed systems and real robot scenarios
4. [optional] Run the logger: `rosrun robotino_simulations rtabmap_eval.py`. This will log odometry, loop closures, map entropy, explored space, wheels rotations in different "npy" files saved on the node running location.
5. Run the active node: `roslaunch active_slam active_node.launch`.
    This will launch the [frontier extraction](https://github.com/eliabntt/active_v_slam/blob/master/src/active_slam/launch/frontier_exploration.launch) and the [map information optimizer](https://github.com/eliabntt/active_v_slam/blob/master/src/active_slam/launch/map_extract.launch).
6. [optional] Run the features' follower: `roslaunch robotino_camera_heading best_heading.launch`. The camera specifics are placed in `robotino_simulations/config/camera.yaml`
7. The FSM is launched with `roslaunch robotino_fsm robotino_fsm.launch kind:=2`.
    - With `kind` you can select the utility [see the paper for reference]:
      - 2: Shannon
      - 5: Dynamic weight with obstacles
      - 7: Interpolation
      - 9: Shannon with obstacles
    - With `only_last:=true/false` you can decide if the system should use only the last waypoint to chose the path or use the whole information.
    - With `weighted_avg:=true/false` you can switch between weighted avg and weighted sum as total utility of a path
    - With `pre_fix:=true/false` you can decide if the system should pre-optimize the heading after choosing the path to follow. Only applicable if `only_last:=true`
    - With `mid_optimizer:=true/false` you can turn on/off the optimization procedure for the next waypoint.
