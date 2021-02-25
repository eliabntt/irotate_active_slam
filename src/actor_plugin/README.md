To use the actor plugin,

1. Build the plugin
```
$ mkdir build
$ cd build 
$ cmake ..
$ make
```

2. add the build folder to gazebo plugins. Execute the following line in each new terminal or add it to your .bashrc file

```
$ export GAZEBO_PLUGIN_PATH=<path to MAVOCAP>/Gazebo_Plugins/build:{$GAZEBO_PLUGIN_PATH}
```