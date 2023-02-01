# Offline Global Body Planner

## Overview

This package implements a offline global body planning framework for quadrupedal navigation. Reads a 2D offline global body plan (i.e. motion trajectory).

### Unit Tests

Run the unit test with

```
catkin run_tests offline_global_body_planner
```
## Config files
- **offline_global_body_planner.yaml** Sets planning hyperparameters. Additionally, hyperparameter to select which offline plan to run


## Nodes

### offline_global_body_planner

Publishes a offline global body plan to guide the lower level control to the goal state. The node reads a 2D offline body plan in the form of a CSV file and transforms the plan into a 3D quadrupedal motion trajectory.
Generally, the workflow as of right now is: 
1. Use a double integrator system to generate motion trajectories (i.e. trajectory of states such as pos, vel, and acceleration in the 2D plane) in MATLAB
2. Load into a CSV file using the load2DPlanToCSV.m
3. Change the location to read state and control trajectory for global body plan in yaml file 
4. Execute offline global body plan by running the following commands (ensure IC is the same when spawning):

```
roslaunch quad_utils quad_gazebo.launch world:=flat_20_20 init_pose:=<initial pose of offline plan>
rostopic pub /robot_1/control/mode std_msgs/UInt8 "data: 1"
roslaunch quad_utils quad_plan.launch reference:=offl_gbpl_2
```

where <initial pose of offline plan> is typically "-x -9 -y 1 -z 0.5 -Y 0.0" as of right now.




