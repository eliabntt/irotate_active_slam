# iRotate: an Active Visual SLAM Approach

### [iRotate: Active Visual SLAM for Omnidirectional Robots](https://arxiv.org/abs/2103.11641) --- Published in Robotics and Autonomous Systems 
### [Active Visual SLAM with Independently Rotating Camera](https://ieeexplore.ieee.org/document/9568791) --- Published in ECMR2021 

Check our teasers [here](https://www.youtube.com/watch?v=YFD80TxXghk) and [here](https://www.youtube.com/watch?v=syIYP-uxyg0) to get a quick overview of the project.

__________
![](https://user-images.githubusercontent.com/19806758/109616778-dca1b380-7b35-11eb-8071-be8229fbb127.png)
__________

This repository contains the code of **iRotate**, a three-layered active V-SLAM method for ground robots.

It combines concept of long term planning (frontier exploration), viewpoint refinement (Next-Best-View, Receiding Horizon planners), and online refinement (feature tracking) in a seemless and structured framework. This results in a continuos robot's heading refinement that is always based on the latest map's entropy and feature information. Thanks to that, we are able to fully explore an environment with paths that are up to **39%** shorter!

To allow a continuous rotational movement the system was first developed with an omnidirectional platform, our _Robotino_, that allowed us to avoid kinematic constraints and a free 6DOF planning.

That was a huge limitation. Therefore, with the use of an independent camera rotation mechanism, we extended our algorithm to different non-omnidirectional ground robots.

This project has been developed within [Robot Perception Group](https://ps.is.tue.mpg.de/research_fields/robot-perception-group) at the Max Planck Institute for Intelligent Systems, Tübingen.

#### The published papers can be found in open acces: [iRotate](https://arxiv.org/abs/2103.11641) and [Independent Camera](https://ieeexplore.ieee.org/document/9568791) -- please [cite]() us if you find this work useful

As a bonus, [here](https://github.com/eliabntt/gazebo_three_wheel_omni_plugin) you can find a gazebo controller for omnidirectional robots.

__________
#### The code can be run in both ROS-Melodic/Ubuntu 18.04 and ROS-Noetic/Ubuntu 20.04
__________

## Getting Started

These instructions will help you to set up both the simulation environment and a real robot scenario for development and testing purposes. 

## Branches description
 - the `master` branch can be used to run the simulation environment with omnidirectional and semi-holonomic robot. You can just change the NMPC config with your limits for the velocities of the robot base and for the camera rotation speed
 - the `merged_odometry` branch is the master branch with the proposed combination for the state estimate
 - the `non-holonomic-mpc` is the master branch with the necessary adaptation to simulate a non-holonomic robot within our system. Please note that this is just an approximation. **To use this with the merged odometry strategy** please merge manually the two branches.
 - the two `real_robot` and `real_robot_merged` branches contain the edits to run the master branch with our platform without/with our proposed combined state estimate
 - the `noetic` brach has the edits to run the other branches in Ubuntu 20.04 / ROS Noetic


## Installation

Please check the detailed instructions [here](https://github.com/eliabntt/irotate_active_slam/blob/master/INSTALL.md)

## How to run

You can find more detailed instructions [here](https://github.com/eliabntt/irotate_active_slam/blob/master/RUNNING.md). The following is just an example of what we have run during the experiments. Not all commands are mandatory.
**COMMAND LIST EXAMPLE**

```
roslaunch robotino_simulations world.launch name:="cafe" gui:=true
roslaunch robotino_mpc robotino_mpc.launch 
roslaunch robotino_simulations rtabmap.launch delete:=-d
[optional, recorder] rosrun robotino_simulations rtabmap_eval.py  
[optional, timer] python src/pause.py && ./back.sh 'main_folder' 'experiment_name' 'session_number' #or python3
roslaunch active_slam active_node.launch
roslaunch robotino_camera_heading best_heading.launch 
roslaunch robotino_fsm robotino_fsm.launch kind:=7 only_last_set:=false pre_fix:=false mid_optimizer:=false weighted_avg:=false
```

## Custom robot system

You might want to run the code on your own robot. [Here](https://github.com/eliabntt/irotate_active_slam/blob/master/CUSTOM_ROBOT.md) you can find some hints on what needs to be adapted. 

## Evaluation

### Reproducing results
Results achieved in real world experiments always depend on the hardware in question as well as environmental factors on the day of experiment. However our simulated experiments results were averaged over a large number of identical experiments and should be reproducible by third parties. 

We have uploaded our data [here](https://keeper.mpdl.mpg.de/d/89f292ac267247df826f/) for _iRotate_ experiments, and [here](https://keeper.mpdl.mpg.de/d/fd9cbe7ec0df43c7831c/) for the _independent camera rotation_ ones. 

If you want to keep track of your own results you may want to check [this](https://github.com/eliabntt/irotate_active_slam/blob/master/RESULTS.md) document that contains our evaluation procedure.

## Included external packages / sources
- [pgm_map_creator](https://github.com/hyfan1116/pgm_map_creator)
- [rtabmap_ros](https://github.com/introlab/rtabmap_ros): edited in a couple of source files and froze
- evaluation of the map and frontier extractor from [crowdbot_active_slam](https://github.com/ethz-asl/crowdbot_active_slam).
**    Frontier extractor has been edited to obtain more frontier points.**
- trajectory evaluation from [TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools)
- `realsense-ros` and `realsense-gazebo-plugin` folders from the respective repos ([here](https://github.com/IntelRealSense/realsense-ros/) and [here](https://github.com/pal-robotics/realsense_gazebo_plugin). Note that those have slightly edited tfs. Albeit this shouldn't impact results and you can [probably] freely update those, we cannot guarantee that.

## Citation

If you find this work useful, or you use part of the developed code, please cite us. 

**Note: the following will be updated soon with the RAS - Elsevier citation **
```
@misc{bonetto2021irotate,
    title={iRotate: Active Visual SLAM for Omnidirectional Robots},
    author={Elia Bonetto and Pascal Goldschmid and Michael Pabst and Michael J. Black and Aamir Ahmad},
    year={2021},
    eprint={2103.11641},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
```

```
@INPROCEEDINGS{9568791,  author={Bonetto, Elia and Goldschmid, Pascal and Black, Michael J. and Ahmad, Aamir},  booktitle={2021 European Conference on Mobile Robots (ECMR)},   title={Active Visual SLAM with Independently Rotating Camera},   year={2021},  volume={},  number={},  pages={1-8},  doi={10.1109/ECMR50962.2021.9568791}}
```
## License

All Code in this repository - unless otherwise stated in local license or code headers is

Copyright 2021 Max Planck Institute for Intelligent Systems, Tübingen.

Licensed under the terms of the GNU General Public Licence (GPL) v3 or higher. See: https://www.gnu.org/licenses/gpl-3.0.en.html
