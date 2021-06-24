This folder contains code to Automatically generate an MPC controller using ACADO toolkit.

The file nmpc_solver_setup.cpp is what defines the MPC controller and it is this you should change in order the change the controller. Then build/install it using the explanation below. As an example it easy to change the prediction horizon if that is prefered.

In your ~/.bashrc add the line:
source [pathToAcado]/build/acado_env.sh

Build/Install: 

From this folder do:

```mkdir build
cd build
cmake ..
make
```
The executable is now placed in the folder active_v_slam/src/robotino_mpc/solver_made_from_cpp/solver

```
cd active_v_slam/src/robotino_mpc/solver_made_from_cpp/solver
./nmpc_solver_setup
```

Then the controller is generated.



