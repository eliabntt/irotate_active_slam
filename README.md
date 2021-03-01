# iRotate: Active Visual SLAM for Omnidirectional Robots
This repository contains the code of iRotate, an active V-SLAM method presented in.

## Getting Started

These instructions will help you to set up both the simulation environment and a real robot scenario for development and testing purposes. 

### Installation

There are many prerequisites. Please follow instructions on the linked webpages on how to install them.
- clone this repository
- CMake latest version: follow [this](https://apt.kitware.com/). Tested with CMake 3.16.5
- OpenCV with contrib and non-free enabled. Tested with 3.4.3
- Install ros-melodic-desktop-full following [this](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Install ros prerequisites `sh ros-req.sh`
- (For the notebook visualization) Install python3, pip3 and virtualenv
    - create a virtualenv with python3.6
    - Install in the virtualenv with `pip install -r requirements.txt` the required [packages](https://github.com/eliabntt/active_v_slam/blob/master/requirements.txt)
    - Run `jupyter labextension install jupyterlab-plotly` (if nodejs gives problems run `curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash – && sudo apt-get install nodejs`)
- Install ACADO from [sources](https://acado.github.io/install_linux.html) and follow [this](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_mpc/solver_made_from_cpp/README.md) README
- Install [GTSAM4.x](https://gtsam.org/get_started/) either from sources or APT. **NOTE** be sure that march_native is off. If problems arise during the compilation of RTABMap check [this](https://github.com/introlab/rtabmap_ros/issues/291)
- Install [libpointmatcher](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/CompilationUbuntu.md)
- RTABMap:
    - `git clone https://github.com/introlab/rtabmap && cd rtabmap`
    - `git checkout 39f68c44c`
    - copy the edited rtabmap files into the cloned folder `cp <your-active_v_slam>/rtabmap-edited/* <your-rtabmap>/corelib/`
    - build and install rtabmap. **NOTE** Be careful that the end of `cmake ..` must be "Build files have been written to ..." w/o ANY _subsequent_ warnings. GTSAM, g2o, OpenCV should be automatically recognized and enabled for the building.
    - Test the installation by running `rtabmap` command in a console. A `sudo ldconfig` might be necessary.
 
- Gazebo: 
    - Launch gazebo at least once so that folders are created.
    - Download gazebo's models and place them in `~/.gazebo/models` (at least the sun one, others should be automatically downloaded when needed).
    - Download 3dgems models with `sh 3dgems.sh`
    - Download models from [AWS](https://github.com/aws-robotics/aws-robomaker-small-house-world/tree/ros1/models) and place them in `~/.gazebo/models`
- Catkin make on  the project folder
______
- **ONLY REAL ROBOT**: 
    - Install [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
    - robotino's packages: robotino-dev rec-rpc and robotino-api2 from [here](https://wiki.openrobotino.org/index.php?title=Robotino_OS)
    - REMOVE the CATKIN_IGNORE from `src/realsense-ros/realsense2-camera`, `src/robotino-node`, `src/robotino-rest-node`, `src/robotino-teleop`
    - **Needs testing ** `mkdir -p ./devel/include/robotino_msgs && cp robotino_specific/robotino_msgs/*.h ./devel/include/robotino_msgs`
_______
## How to run
Since you will need at least 5 terminals to run this project my suggestion is to use a package like _terminator_.
1. Launch the simulation environment `roslaunch robotino_simulations world.launch`
    1. Select the desired environment with `name:=<arg>`. We tested our work with `name:="cafe"` and `name:="small_house" x:=-2.0`.
       
       .world files are placed in `robotino_simulations/worlds/`
    2. This node takes care also of launching the nodes for generating the necessary ground truth transforms
    3. In case a shift is needed (like with the AWS's small_house) the `robot_ekf` in `robotino_description/launch/robotino.launch` should be adapted to get the corrispondent starting pose
2. Run the MPC:  `roslaunch robotino_mpc robotino_mpc.launch`. Parameters are being set in `robotino_mpc/cfg/nmpc.yaml`.

    NMPC constraints can be configure either dynamically with dynamic configure or by editing the `robotino_mpc/cfg/NonLinearMPC.cfg` file. Note that the latter require a `catkin_make` afterwards.
3. Run RTABMap: `roslaunch robotino_simulations rtabmap.launch`.
    1. the argument `delete:=-d` lets the user delete the previously generated db at launch
    2. the default folder of the db is `.ros/rtabmap.db` (setted with the `db` argument)
    3. RTABMap parameters can be edited directly in `robotino_simulations/launch/mapping.launch`
    4. RTABMapViz (the visualization tool of RTABMap) is currently commented in `robotino_simulations/launch/mapping.launch`. Note that with this enabled performance will suffer most probably requiring RTABMap to use its memory management techniques.
    5. This node will launch also `move_base` and the costmap converter. The parameters are placed in `robotino_simulations/config`
4. [optional] Run the logger: `rosrun robotino_simulations rtabmap_eval.py`. This will log odometry, loop closures, map entropy, explored space, wheels rotations in different "npy" files saved on the node running location.
5. Run the active node: `roslaunch active_slam active_node.launch`.
    This will launch the frontier extraction and the map information optimizer.
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

## Evaluation
To generate evaluation files you need to edit the following files `map.sh`, `back.sh`, `robotino_simulations/src/convert_map.cpp` and `robotino_simulations/src/calculate_map_error.cpp` such that folders are correct for your system.
- `convert_map.cpp:L12` is the destination folder of the map converted to a txt file.
- `calculate_map_error.cpp:L10-14` are the folders/files used to generate the results and read the (gt)map
- `map.sh` will be then use to create the map, compute the results and generate the log files from rtabmap database
- `back.sh` will run the trajectory evaluation scripts and `map.sh` and move all the files to the desired folder

The groundtruths for the considered environments (generated with the pgm_map_creator) are in `src/pgm_map_creator/maps`.


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

### Included external packages / sources
- [pgm_map_creator](https://github.com/hyfan1116/pgm_map_creator)
- [rtabmap_ros](https://github.com/introlab/rtabmap-ros): edited in a couple of source files and froze
- evaluation of the map and frontier extractor from [crowdbot_active_slam](https://github.com/ethz-asl/crowdbot_active_slam). 
  
    Frontier extractor has been edited to obtain more frontiers.
- trajectory evaluation from [TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools)
- realsense-ros and realsense-gazebo-plugin folders. Note that those have slightly edited tfs. This won't impact results you can [probably] freely update those.
## Authors

**Elia Bonetto** 
## License

This project is licensed under
