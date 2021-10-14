# iRotate: an Active Visual SLAM Approach

### [iRotate: Active Visual SLAM for Omnidirectional Robots](https://arxiv.org/abs/2103.11641) --- [video](https://www.youtube.com/watch?v=YFD80TxXghk)
### [Active Visual SLAM with Independently Rotating Camera](https://arxiv.org/abs/2105.08958) --- [video](https://www.youtube.com/watch?v=syIYP-uxyg0)
__________
#### Available for both ROS-Melodic/Ubuntu 18.04 and ROS-Noetic/Ubuntu 20.04
__________

This repository contains the code of iRotate, an active V-SLAM method submitted to IROS2021.
Moreover, it is now possible to use our independent camera rotation and also run the method to non-omnidirectional robotic platforms. For details of this work check the corresponding paper submitted to ECMR2021.

This project has been developed within [Robot Perception Group](https://ps.is.tue.mpg.de/research_fields/robot-perception-group) at the Max Planck Institute for Intelligent Systems, Tübingen.

#### The papers can be found [here](https://arxiv.org/abs/2103.11641)(iRotate) and [here](https://arxiv.org/abs/2105.08958)(independent camera) -- please cite us if you find this work useful

![](https://user-images.githubusercontent.com/19806758/109616778-dca1b380-7b35-11eb-8071-be8229fbb127.png)

## Getting Started

These instructions will help you to set up both the simulation environment and a real robot scenario for development and testing purposes. 

## Branches description
 - the `master` branch can be used to run the simulation environment with omnidirectional and semi-holonomic robot. You can just change the NMPC config with your limits for the velocities of the robot base and for the camera rotation speed
 - the `merged_odometry` branch is the master branch with the proposed combination for the state estimate
 - the `non-holonomic-mpc` is the master branch with the necessary adaptation to simulate a non-holonomic robot within our system. Please note that this is just an approximation.
 - the two `real_robot` and `real_robot_merged` branches contain the edits to run the master branch with our platform without/with our proposed combined state estimate
 - the `noetic` brach has the edits to run the other branches in Ubuntu 20.04 / ROS Noetic


## Installation

There are many prerequisites. Please follow instructions on the linked webpages on how to install them.
- Clone this repository. 
     - **IMPORTANT: move the content of the `src` folder in the `src` folder of your catkin workspace**
- Update CMake to the latest version: follow [this](https://apt.kitware.com/). Tested with CMake 3.16.5
- OpenCV with contrib and non-free enabled. Tested with 3.4.3
- Install ros-melodic-desktop-full following [this](http://wiki.ros.org/melodic/Installation/Ubuntu)
  - **If you want to use ros-NOETIC switch branch**
- Install ros prerequisites `sh ros-req.sh`
- \[optional\] (For the notebook visualization) Install python3, pip3 and virtualenv
    - create a virtualenv with python3.6
    - Install in the virtualenv with `pip install -r requirements.txt` the required [packages](https://github.com/eliabntt/active_v_slam/blob/master/requirements.txt)
    - Run `jupyter labextension install jupyterlab-plotly` (if nodejs gives problems run `curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash – && sudo apt-get install nodejs`)
- Install ACADO from [sources](https://acado.github.io/install_linux.html) and follow [this](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_mpc/solver_made_from_cpp/README.md) README
- Install [GTSAM4.x](https://gtsam.org/get_started/) either from sources or APT. **NOTE** If building from sources be sure that march_native is [off](https://github.com/rvaser/spoa/issues/20), or, as an alternative, you can have g2o+PCL+GTSAM all built from sources with such option enabled. If problems arise during the compilation of RTABMap check [this](https://github.com/introlab/rtabmap_ros/issues/291)
- Install [libpointmatcher](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/CompilationUbuntu.md)
- RTABMap:
    - `git clone https://github.com/introlab/rtabmap && cd rtabmap`
    - `git checkout 39f68c44c`
    - copy the edited rtabmap files into the cloned folder `cp <your-active_v_slam>/rtabmap-edited/* <your-rtabmap>/corelib/src`
    - build and install rtabmap. **NOTE** Be careful that the end of `cmake ..` must be "Build files have been written to ..." w/o ANY _subsequent_ warnings. GTSAM, g2o, OpenCV should be automatically recognized and enabled for the building.
    - Test the installation by running `rtabmap` command in a console. A preemptive `sudo ldconfig` might be necessary.
 
- Gazebo: 
    - Launch gazebo at least once so that folders are created.
    - Get the models
        - \[Option 1\] Download and follow instructions for models from [here](https://github.com/eliabntt/gazebo_models)
        - \[Option 2\] download models from 3dgems with `sh 3dgems.sh` -- **note** it seems down -- and from [AWS](https://github.com/aws-robotics/aws-robomaker-small-house-world/tree/ros1/models) and place them in `~/.gazebo/models`
- Catkin make on the main project folder
______
- **ONLY REAL ROBOT**: 
    - Install [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
    - robotino's packages: rec-rpc robotino-dev and robotino-api2 from [here](https://wiki.openrobotino.org/index.php?title=Robotino_OS) in this order
    - REMOVE the CATKIN_IGNORE from `src/realsense-ros/realsense2-camera`, `src/robotino-node`, `src/robotino-rest-node`, `src/robotino-teleop`
    - do `catkin_make` again. If error occurs because of missing robotino's msg files you can run `mkdir -p ./devel/include/robotino_msgs && cp robotino_specific/robotino_msgs/*.h ./devel/include/robotino_msgs`

## How to run
Since you will need at least 5 terminals to run this project my suggestion is to use a package like _terminator_.
1. Launch the simulation environment `roslaunch robotino_simulations world.launch` [here](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_simulations/launch/world.launch). 
    1. Select the desired environment with `name:=<arg>`. We tested our work with `name:="cafe"` and `name:="small_house" x:=-2.0`.
       
       .world files are placed in `robotino_simulations/worlds/`
    2. This node takes care also of launching the nodes for generating the necessary ground truth transforms
    3. In case a shift is needed (like with the AWS's small_house) the `robot_ekf` in `robotino_description/launch/robotino.launch` should be adapted to get the corrispondent starting pose since groundtruth will reflect this shift
2. Run the [MPC](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_mpc/launch/robotino_mpc.launch):  `roslaunch robotino_mpc robotino_mpc.launch`. [Parameters](https://github.com/eliabntt/active_v_slam/tree/master/src/robotino_mpc/cfg) in `robotino_mpc/cfg/nmpc.yaml` will be used in the code to configure mainly the topics while NMPC constraints can be configured either dynamically (`rosrun rqt_reconfigure rqt_reconfigure`) or by editing the `robotino_mpc/cfg/NonLinearMPC.cfg` file. Note that the latter require a `catkin_make` afterwards.
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
________
**COMMAND LIST EXAMPLE**

```
roslaunch robotino_simulations world.launch name:="cafe" gui:=true
roslaunch robotino_mpc robotino_mpc.launch 
roslaunch robotino_simulations rtabmap.launch delete:=-d
rosrun robotino_simulations rtabmap_eval.py  
python src/pause.py && ./back.sh 'main_folder' 'experiment_name' 'session_number' #or python3
roslaunch active_slam active_node.launch
roslaunch robotino_camera_heading best_heading.launch 
roslaunch robotino_fsm robotino_fsm.launch kind:=7 only_last_set:=false pre_fix:=false mid_optimizer:=false weighted_avg:=false
```

## Custom robot system
This algorithm has been tested thoroughly based on our hw architecture fully simulated inside the Gazebo environment.
We developed a custom Gazebo's [plugin](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_gazebo_plugin/src/robotino_plugin.cpp) to generate odometry and movement of the three-wheel omnidirectional robot system so that we have correct joint movements and representation of the robot real movements.
If you want to test this algorithm on your own system there are some changes you might want to apply:
- camera, IMU, laser scan and odom topics [here](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_simulations/launch/mapping.launch), [here](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_simulations/launch/move_base.launch), [here](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_fsm/src/robotino_fsm_node.cpp) and [here](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_mpc/cfg/nmpc.yaml)
- camera configuration [here](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_simulations/config/camera.yaml)
- Frontier size in the `active_slam` package
- EKF's [settings](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_description/launch/robotino.launch) and **topics**: note that the system relies on two odometry sources, one for the camera wrt the base and one for the base. The system will indeed need both topics to be published to work (i.e. the FSM and the MPC relies on synchronization of odom messages from these two sources).
- Other things like `move_base` setting

## Evaluation

### Reproducing results
Results achieved in real world experiments always depend on the hardware in question as well as environmental factors on the day of experiment. However our simulated experiments results were averaged over a large number of identical experiments and should be reproducible by third parties.

We focused on AWS's Small House and an edited version of the Gazebo's Cafè environments. Tests results are averaged among 20 tries of 10 minutes each with the same starting location ([-2,0] and [0,0] respectively).

No changes to the code are necessary.

### Generate evaluation files and analyze the results
To generate evaluation files you need to edit the following files `map.sh`, `back.sh`, `robotino_simulations/src/convert_map.cpp` and `robotino_simulations/src/calculate_map_error.cpp` such that folders are correct for your system.
- `convert_map.cpp:L12` is the destination folder of the map converted to a txt file.
- `calculate_map_error.cpp:L10-14` are the folders/files used to generate the results and read the (gt)map
- `map.sh` will be then use to create the map, compute the results and generate the log files from rtabmap database
- `back.sh` will run the trajectory evaluation scripts and `map.sh` and move all the files to the desired folder

The groundtruths for the considered environments \[Cafe and AWS's small house\] (generated with the pgm_map_creator) are in `src/pgm_map_creator/maps`.


The suggested folder structure is as follow (since this is the one used in scripts and notebook):
- Test/
  - env/
    - occupancy_gt.txt
    - folder/ [i.e. experiment identifier]
        - 0/ [i.e. run  number ]
        - 1/
        - 2/
        - ...

`folder` should be named as `utility_env_testkind`. For example using Shannon, on env 1 and every component enabled (A) I generated:
`Test/E1/S_E1_A/0`.

**While** running the experiments you need run the logger (step 4).

One can use `python src/pause.py` to pause the simulation after a given amount of time.

After the run has been paused you can call `sh back.sh 'env' 'folder' 'try'` and generate all the results.

**NOTE**: these scripts assume that all the components are available and online [roscore, rtabmap, map server...]

The python notebook can then be used to generate paper-like plots and results.

The BAC score will be computed inside the notebook.

The notebook requires the "poses.g2o" file for each run. This was initially used to gather the evolution of link uncertainties. Even if it's not directly useful I suggest to still export this via `rtabmap-databaseViewer` tool by opening the database and doing `File>export poses> g2o` so that you can double-check the  correctness of every run.

**NOTE** Most probably you will need to edit the notebook based on your need: e.g. length of the experiments, bucketing and other things are managed inside.

## Included external packages / sources
- [pgm_map_creator](https://github.com/hyfan1116/pgm_map_creator)
- [rtabmap_ros](https://github.com/introlab/rtabmap_ros): edited in a couple of source files and froze
- evaluation of the map and frontier extractor from [crowdbot_active_slam](https://github.com/ethz-asl/crowdbot_active_slam). 
  
    Frontier extractor has been edited to obtain more frontiers.
- trajectory evaluation from [TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools)
- realsense-ros and realsense-gazebo-plugin folders. Note that those have slightly edited tfs. This won't impact results you can [probably] freely update those.

## License

All Code in this repository - unless otherwise stated in local license or code headers is

Copyright 2021 Max Planck Institute for Intelligent Systems, Tübingen.

Licensed under the terms of the GNU General Public Licence (GPL) v3 or higher. See: https://www.gnu.org/licenses/gpl-3.0.en.html
