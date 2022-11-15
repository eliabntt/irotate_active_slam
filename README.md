# This version of the code has been developed specifically to be run with the `irotate` branch of our [isaac sim simulation manager](https://github.com/eliabntt/isaac_sim_manager/tree/irotate_edits)
----
# iRotate: an Active Visual SLAM Approach

### [iRotate: Active Visual SLAM for Omnidirectional Robots](https://arxiv.org/abs/2103.11641) --- Published in Robotics and Autonomous Systems 
### [Active Visual SLAM with Independently Rotating Camera](https://ieeexplore.ieee.org/document/9568791) --- Published in ECMR2021 

Check our teasers [here](https://www.youtube.com/watch?v=YFD80TxXghk) and [here](https://www.youtube.com/watch?v=syIYP-uxyg0) to get a quick overview of the project.

__________
![](https://user-images.githubusercontent.com/19806758/109616778-dca1b380-7b35-11eb-8071-be8229fbb127.png)
__________

This repository contains the code of **iRotate**, a three-layered active V-SLAM method for ground robots.

It combines concept of long term planning (frontier exploration), viewpoint refinement (Next-Best-View, Receiding Horizon planners), and online refinement (feature tracking) in a seemless and structured framework. This results in a continuos robot's heading refinement that is always based on the latest map's entropy and feature information. Thanks to that, we are able to fully explore an environment with paths that are up to **39%** shorter!

To allow a continuous rotational movement the system was first developed with an omnidirectional platform, our _Robotino_, that allowed us to avoid kinematic constraints and a free 3DOF planning.

That was a huge limitation. Therefore, with the use of an independent camera rotation mechanism, we extended our algorithm to different non-omnidirectional ground robots.

This project has been developed within [Robot Perception Group](https://ps.is.tue.mpg.de/research_fields/robot-perception-group) at the Max Planck Institute for Intelligent Systems, Tübingen.

#### The published papers can be found in open acces: [iRotate](https://arxiv.org/abs/2103.11641) and [Independent Camera](https://ieeexplore.ieee.org/document/9568791) -- please [cite](https://github.com/eliabntt/irotate_active_slam/blob/master/README.md#Citation) us if you find this work useful

As a bonus, [here](https://github.com/eliabntt/gazebo_three_wheel_omni_plugin) you can find a gazebo controller for omnidirectional robots.

__________


## Citation

If you find this work useful, or you use part of the developed code, please cite us. 

```
@article{BONETTO2022104102,
title = {iRotate: Active visual SLAM for omnidirectional robots},
journal = {Robotics and Autonomous Systems},
pages = {104102},
year = {2022},
issn = {0921-8890},
doi = {https://doi.org/10.1016/j.robot.2022.104102},
url = {https://www.sciencedirect.com/science/article/pii/S0921889022000550},
author = {Elia Bonetto and Pascal Goldschmid and Michael Pabst and Michael J. Black and Aamir Ahmad},
keywords = {View planning for SLAM, Vision-based navigation, SLAM}
}
```

```
@INPROCEEDINGS{9568791,  
author={Bonetto, Elia and Goldschmid, Pascal and Black, Michael J. and Ahmad, Aamir},  
booktitle={2021 European Conference on Mobile Robots (ECMR)},   
title={Active Visual SLAM with Independently Rotating Camera},   
year={2021},  
volume={},  
number={},  
pages={1-8},  
doi={10.1109/ECMR50962.2021.9568791}
}
```
## License

All Code in this repository - unless otherwise stated in local license or code headers is

Copyright 2021 Max Planck Institute for Intelligent Systems, Tübingen.

Licensed under the terms of the GNU General Public Licence (GPL) v3 or higher. See: https://www.gnu.org/licenses/gpl-3.0.en.html
