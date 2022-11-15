There are many prerequisites. Please follow instructions on the linked webpages on how to install them.
- Clone this repository. 
     - **IMPORTANT: move the content of the `src` folder in the `src` folder of your catkin workspace**
- Update CMake to the latest version: follow [this](https://apt.kitware.com/). Tested with CMake 3.16.5
- OpenCV with contrib and non-free enabled. Tested with 3.4.3. Works with 4.* but results were not checked. 
    - If you build with cuda you might need to then install `gcc7` and run `catkin build -DCMAKE_C_COMPILER=$(which gcc-7) -DCMAKE_CXX_COMPILER=$(which g++-7)` with latest version of Ubuntu (adapt it to your cuda version).
 
----

- Install ros-melodic-desktop-full following [this](http://wiki.ros.org/melodic/Installation/Ubuntu)
  - **If you want to use ros-NOETIC remember to switch branch, NOTE it has not been fully tested -- it should work**
     The main differences are the `camera_best_heading` package, some `CMakeLists.txt` and there is the necessity to install the correct `rtabmap` versions. 
     
** YOU SHOULD BE ABLE TO MERGE ANY BRANCH WITH THE NOETIC ONE. IF NO/IN DOUBT PLEASE OPEN AN ISSUE **
     
----

- Install ros prerequisites `sh ros-req.sh`. **In noetic is not fully tested -- it should work**
    - A simpler way might be to run ``` sudo apt install ros-{version}-rtabmap ros-{version}-rtabmap-ros ``` and install any other prerequisite while building the packages. 
     ```
     sudo apt purge ros-{version}-rtabmap ros-{version}-rtabmap-ros 
     ```
     *IF you install rtabmap this way you need to UNINSTALL IT*
- \[optional\] (For the notebook visualization) Install python3, pip3 and virtualenv
    - create a virtualenv with python3.6
    - Install in the virtualenv with `pip install -r requirements.txt` the required [packages](https://github.com/eliabntt/active_v_slam/blob/master/requirements.txt)
    - Run `jupyter labextension install jupyterlab-plotly` (if nodejs gives problems run `curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash – && sudo apt-get install nodejs`)
- Install ACADO from [sources](https://acado.github.io/install_linux.html) and follow [this](https://github.com/eliabntt/active_v_slam/blob/master/src/robotino_mpc/solver_made_from_cpp/README.md) README
- Install [GTSAM4.x](https://gtsam.org/get_started/) either from sources or APT **strongly suggested - stable release**. **NOTE** If building from sources be sure that march_native is [off](https://github.com/rvaser/spoa/issues/20), or, as an alternative, you can have g2o+PCL+GTSAM all built from sources with such option enabled. If problems arise during the compilation of RTABMap check [this](https://github.com/introlab/rtabmap_ros/issues/291)
- Install [libpointmatcher](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/CompilationUbuntu.md)

---

- RTABMap:
    -  OPTION 1 (paper version) with OLDER rtabmap-rtabmap_ros (the one from the master branch)
     ```
     git clone https://github.com/introlab/rtabmap && cd rtabmap
     git checkout 39f68c44c 
     ```
    copy the edited rtabmap files into the cloned folder `cp <your-active_v_slam>/rtabmap-edited/* <your-rtabmap>/corelib/src`
    *Use in catkin workspace the `rtabmap_ros` AND the `camera_best_heading` from the **master** branch*
    
    -  OPTION 2 use the latest version of RTABMap as of **MAY 19 2022**
    ```
    git clone https://github.com/eliabntt/rtabmap && cd rtabmap
    ```
    
    *Use in catkin workspace the `rtabmap_ros` AND the `camera_best_heading` from the **noetic** branch*
    
    *NOTE: if you find problems with this checkout rtabmap at the 5d200a0 commit and `cp <your-active_v_slam>/rtabmap-edited/Rtabmap_5d200a0.cpp <your-rtabmap>/corelib/src/Rtabmap.cpp`
    
    - build and install rtabmap. **NOTE** Be careful that the end of `cmake ..` must be "Build files have been written to ..." w/o ANY _subsequent_ warnings. GTSAM, g2o, OpenCV, libpointmatcher, and octomape should be automatically recognized and enabled for the building.
    - Test the installation by running `rtabmap` command in a console. A preemptive `sudo ldconfig` might be necessary.
  
----

- Gazebo: 
    - Launch gazebo at least once so that folders are created.
    - Get the models
        - \[Option 1\] Download and follow instructions for models from [here](https://github.com/eliabntt/gazebo_models)
        - \[Option 2\] download models from 3dgems with `sh 3dgems.sh` -- **note** it seems down -- and from [AWS](https://github.com/aws-robotics/aws-robomaker-small-house-world/tree/ros1/models) and place them in `~/.gazebo/models`
- Catkin make (both `catkin build` or `catkin_make` works) on the main project folder.
  With the newer rtabmap you might need `catkin build  -DRTABMAP_SYNC_USER_DATA=ON` since we use the synchronization module.
