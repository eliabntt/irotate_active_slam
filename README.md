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
A step by step series of examples that tell you how to get a development env running
- **ONLY REAL ROBOT**: 
    - [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
    - robotino's packages: robotino-dev rec-rpc and robotino-api2 from [here](https://wiki.openrobotino.org/index.php?title=Robotino_OS)
    - REMOVE the CATKIN_IGNORE from `src/realsense-ros/realsense2-camera`, `src/robotino-node`, `src/robotino-rest-node`, `src/robotino-teleop`
    - **Needs testing ** `mkdir -p ./devel/include/robotino_msgs && cp robotino_specific/robotino_msgs/*.h ./devel/include/robotino_msgs`
   
## How to run

## Authors

* **Elia Bonetto** 
* 
## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
