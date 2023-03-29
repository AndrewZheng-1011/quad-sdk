#!/usr/bin/env python

import roslaunch
import rospy
import numpy as np
import time

# Start position for world:="full_terrain_map" init_pose:="-x 7 -y 0.9 -z 0.5 -Y 3.3"

print("Executing waypoint plan...")
time.sleep(1)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

print("Starting 1st waypoint")
launch_args = ['quad_utils', 'quad_plan.launch']
launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
launch.start()
time.sleep(25)
launch.shutdown()

print("Start 2nd waypoint...")
launch_args = ['quad_utils', 'quad_plan.launch', "goal_plan:=2"]
launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
launch2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
launch2.start()
time.sleep(110)
launch2.shutdown()

print("Start 3rd waypoint...")
launch_args = ['quad_utils', 'quad_plan.launch', "goal_plan:=3"]
launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
launch3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
launch3.start()
time.sleep(85)
launch3.shutdown()

print("Start 4th waypoint...")
launch_args = ['quad_utils', 'quad_plan.launch', "goal_plan:=4"]
launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
launch4 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
launch4.start()
time.sleep(135)
launch4.shutdown()

print("Start 5th waypoint...")
launch_args = ['quad_utils', 'quad_plan.launch', "goal_plan:=5"]
launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
launch5 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
launch5.start()
time.sleep(200)
launch5.shutdown()
print("Goal!")