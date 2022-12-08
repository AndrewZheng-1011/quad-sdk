# Multi-Link Tail Controller Branch

This repository is inspired from the original repo taken from this paper: https://www.andrew.cmu.edu/user/amj1/papers/Proprioception_and_Tail_Control_ICRA_WS.pdf

The following is a small project based off this paper. This work uses open loop controls to help re-orient quadruped walking off cliff

## Prerequisite
Please follow the main branch on how to download quad-sdk. Then you can switch to this branch to run the following scripts below.

## Launch Files
The following branch can also run through the roslaunch files:

```
roslaunch quad_utils quad_gazebo_dira.launch tail:=true tail_type:=4 tail_num:=2 dira:=true gui:=true
```

## Script File
To run the following program through a bash script file, run either:

```
./run_batch_simulation_single.py 11 0 3 true 2 # Option 1

./run_batch_simulation_single.py 11 40 3 true 2 '45.0' '-45.0' '4.75' '5.0' # Option 2
```
where the following parameters are:
1) world (e.g. 11 runs step_80cm world)
2) init_pos
3) tail_type (3 is for the open loop control)
4) Enable DIRA model (Modified version of the spirit robot)
5) tail_num (e.g. 2 tails | There are tail links of number 1,2,3,4,5,6, and 10)
6) ff_torque_0 (Torque for 1st open-loop control)
7) ff_torque_1 (Torque for 2nd open-loop control)
8) time_0 (Time for 1st open-loop control)
9) time_1 (Time for 2nd open-loop control)

Or, run a batch script of curated open-loop torque control

```
./run_batch_simulation_dira.py
```

## Remarks
Note, that tail_controller.yaml has the parameters to change the open-loop torque value and the time to execute these. Following can be accessed through the following directory as shown below:

```
roscd tail_controller/
gedit tail_controller.yaml
```

## TODO
Add constant FF torque for multi-link tail s.t. continuum tail can orient upwards

