#!/usr/bin/env python
import roslaunch
import rospy
import numpy as np
import sys


world_index = int(sys.argv[1])
batch_index = int(sys.argv[2])
type_index = int(sys.argv[3])

print("world index: %d, batch_index: %d, type_index: %d" %
      (world_index, batch_index, type_index))

vel = 1.0
period = 0.36
num = 100
time_init = 3.5/4*10 * 2  # Change this if sim too long to run
time_stand = 7.5/4*10
time_walk = 27.5/4*10*0.75/1
world = ['world:=step_25cm', 'world:=step_30cm', 'world:=step_35cm', 'world:=step_40cm',
         'world:=step_45cm', 'world:=step_50cm', 'world:=step_55lcm', 'world:=step_60cm',
         'world:=step_65cm', 'world:=step_70cm', 'world:=step_75cm', 'world:=step_80cm']

np.random.seed(0)

# init pose calculation s.t. they can ensure they fall
init_pos = np.linspace(-vel*period/2, vel*period/2,
                       num, endpoint=False) + 6.0


if type_index == 0:
    # Leg
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch_args = ['quad_utils', 'quad_gazebo.launch', 'paused:=false', 'rviz_gui:=true', 'gui:=true',
                   world[world_index], 'x_init:='+str(init_pos[batch_index])]
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch.start()
    rospy.loginfo('Gazebo running')

    print("NO TAIL SCENARIO")
    print("SLEEPING FOR %d" % time_init)
    rospy.sleep(time_init)

    launch_args = ['quad_utils', 'standing.launch']
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_2.start()
    rospy.loginfo('Standing')

    print("TIME TO STAND %0.2f" % time_stand)
    rospy.sleep(time_stand)

    launch_args = ['quad_utils', 'planning.launch',
                   'logging:=true', 'parallel_index:='+str(batch_index)]
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_3.start()
    rospy.loginfo('MPC running')

    print("TIME FOR WALKING: %0.2d" % time_walk)
    rospy.sleep(time_walk)

    launch.shutdown()
    launch_2.shutdown()
    launch_3.shutdown()
elif type_index == 1:
    # Tail
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch_args = ['quad_utils', 'quad_gazebo.launch', 'paused:=false', 'rviz_gui:=true',  'gui:=true',
                   world[world_index], 'tail:=true', 'tail_type:=2', 'x_init:='+str(init_pos[batch_index])]
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch.start()
    rospy.loginfo('Gazebo running')
    print("TIME TO INITIALIZE: %0.2f" % time_init)
    rospy.sleep(time_init)

    launch_args = ['quad_utils', 'standing.launch']
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_2.start()
    rospy.loginfo('Standing')

    print("TAIL MPC SCENARIO")
    print("TIME TO STAND %0.2f" % time_stand)
    rospy.sleep(time_stand)

    launch_args = ['quad_utils', 'planning.launch',
                   'logging:=true', 'tail:=true', 'parallel_index:='+str(batch_index)]
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_3.start()
    rospy.loginfo('MPC running')

    print("TIME FOR WALKING: %0.2d" % time_walk)
    rospy.sleep(time_walk)

    launch.shutdown()
    launch_2.shutdown()
    launch_3.shutdown()
elif type_index == 2:
    # Feedback Tail
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch_args = ['quad_utils', 'quad_gazebo.launch', 'paused:=false', 'rviz_gui:=true',
                   world[world_index], 'tail:=true', 'tail_type:=3', 'x_init:='+str(init_pos[batch_index])]
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch.start()
    rospy.loginfo('Gazebo running')

    print("FEEDBACK/DECENTRALIZED TAIL SCENARIO")
    print("TIME TO INITIALIZE: %0.2f" % time_init)
    rospy.sleep(time_init)

    launch_args = ['quad_utils', 'standing.launch']
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_2.start()
    rospy.loginfo('Standing')

    print("TIME TO STAND %0.2f" % time_stand)
    rospy.sleep(time_stand)

    launch_args = ['quad_utils', 'planning.launch',
                   'logging:=true', 'tail:=false', 'parallel_index:='+str(batch_index)]
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_3.start()
    rospy.loginfo('MPC running')

    print("TIME FOR WALKING: %0.2d" % time_walk)
    rospy.sleep(time_walk)

    launch.shutdown()
    launch_2.shutdown()
    launch_3.shutdown()


elif type_index == 3:
    # Feedback Tail
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch_args = ['quad_utils', 'quad_gazebo.launch', 'paused:=false', 'rviz_gui:=true',
                   world[world_index], 'tail:=true', 'tail_type:=4', 'x_init:='+str(init_pos[batch_index])]
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch.start()
    rospy.loginfo('Gazebo running')

    print("OPEN LOOP TAIL SCENARIO")
    print("TIME TO INITIALIZE: %0.2f" % time_init)
    rospy.sleep(time_init)

    launch_args = ['quad_utils', 'standing.launch']
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_2.start()
    rospy.loginfo('Standing')

    print("TIME TO STAND %0.2f" % time_stand)
    rospy.sleep(time_stand)

    launch_args = ['quad_utils', 'planning.launch',
                   'logging:=true', 'tail:=false', 'parallel_index:='+str(batch_index)]
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_3.start()
    rospy.loginfo('MPC running')

    print("TIME FOR WALKING: %0.2d" % time_walk)
    rospy.sleep(time_walk)

    launch.shutdown()
    launch_2.shutdown()
    launch_3.shutdown()
