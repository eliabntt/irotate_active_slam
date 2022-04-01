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
