^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pid
^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.28 (2020-07-03)
-------------------
* Fix compatibility issues when building with ROS Noetic
* Bug fix for angle wrapping always resetting filtered and integral error
* Adding a parameter setpoint_timeout which specifies whether to keep publishing control_effort messages after setpoint messages stop coming in
* Add link to low-pass filter reference
* Cleaning up package.xml just a bit more.
* Removing a bunch of superfluous dependencies.
* Cleaning CMakeLists.
* Upgrade to package format=2
* Contributors: Andrew J Zelenak, AndyZe, Stewart Jamieson, Vikrant Shah

0.0.15 (2016-02-22)
-------------------
* Fixing header "include" error, possible dynamic_reconfigure errors.
* Contributors: Andy Zelenak

0.0.10 (2016-1-25)
------------------
* Add diagnostics running at 4Hz. Publish interesting data like setpoint, plant state,
error, control effort in diags for viewing
* Add ROS private parameters to set Pid params and simulator param
* Add Auto/Manual mode: Listen to the /pid_enable topic for a std_msgs/Bool
that will disable or re-enable output from the PID controller
* Support reverse_acting parameter, for plants where the control input acts
in the opposite direction to the plant-state. E.g. Differential-drive robots
often have one direct and one reverse-acting PID
* Support faster-than-wallclock simulation via a /clock publisher and the
/use_sim_time parameter
* Add launch files for first & 2nd order behaviors. Get them to launch plots,
diag monitor, reconfigure gui
* Split setpoint generator out into a separate node
* Rename simulator to plant_sim.cpp & give it 1st & 2nd order behaviors, configured
with a parameter. 
* Delete first_order_plant_sim.cpp, which is subsumed into plant_sim.cpp. Remove
plant header, which was almost content-free.
* Remove msg directory & switch to using Float64 messages for setpoint,
process state, & control effort. Now it's generic - no special messages needed.
* Explicitly call out std namespace to avoid accidental name conflicts
* Remove parameter length checks to allow parameters to be set in part from
launch file (the usual way), and in part from cmd line args (an infrequently-used
way)
* Rename pid_header.h to controller.h because it's used by controller.cpp - more standard
C++ naming style
* Add copyrights
* Contributors: AndyZe, Paul Bouchier

0.0.9 (2015-12-27)
------------------
* Merged in bouchier/pid (pull request #1)
  Add dynamic reconfigure for Kx, change plant setpoint
* add dynamic reconfigure for Kx, change plant setpoint
* Contributors: AndyZe, Paul Bouchier

0.0.7 (2015-07-26)
------------------
* Added a launch file, which required the arguments to be processed differently.
* Contributors: Andy Zelenak

0.0.6 (2015-06-09)
------------------
* Changing the way parameters are passed to check_user_input().
* Contributors: Andy Zelenak

0.0.5 (2015-06-09)
------------------
* Adding an anti-windup option.
* Contributors: Andy Zelenak

0.0.3 (2015-03-14)
------------------

0.0.2 (2015-03-13)
------------------

0.0.1 (2015-03-08)
------------------
* Fixing various minor bugs related to user input.
* Pre-release commit.
* It WORKS!
* It's talking with the plant sim now. Just the PID programming remains.
* Making progress with the command line input.
* Rough outline of the program. Need to take inputs on the command line next.
* Initial commit
* Contributors: Andy Zelenak
