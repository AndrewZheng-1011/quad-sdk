#!/usr/bin/env python
import roslaunch
import rospy
import numpy as np
import time

vel = 1.0
period = 0.48
sample = 100
time_init = 3.5/4*10 * 2  # Change this if sim/computer slow
time_stand = 7.5/4*10
time_walk = 27.5/4*10*0.75/1
world = 'world:=step_80cm'
# Set negative values to time not have torque
batch_num = [0, 40, 40, 40, 40, 40]
ff_torque_1 = ['20', '20', '30', '0', '30', '30']
time_1 = ['4.50', '4.75', '4.75', '-2', '4.75', '5.1']
ff_torque_2 = ['0', '0', '0', '0', '-45', '-15']
time_2 = ['-2', '-2', '-2', '-2', '5.00', '5.3']
tail_num = ['1', '1', '1', '2', '2', '2']

np.random.seed(0)

init_pos = np.linspace(-vel*period/2, vel*period/2,
                       sample, endpoint=False) + 5.0

num_repeat = 10
# init_pos = np.array([init_pos]*int(num/sample)).T.flatten()
# init_pos = [init_pos[i] for i in [15]]
# print(init_pos)


for i in range(len(ff_torque_1)):
    for j in range(num_repeat):
        print("tail_num: %s | ff_torque_1: %s | time_1: %s | ff_torque_2: %s | time_2: %s" %
              (tail_num[i], ff_torque_1[i], time_1[i], ff_torque_2[i], time_2[i]))
        time.sleep(5)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_args = ['quad_utils', 'quad_gazebo_dira.launch', 'paused:=false', 'rviz_gui:=true', 'gui:=true', 'dira:=true',
                       world, 'tail:=true', 'tail_type:=4', 'tail_num:=' +
                       str(tail_num[i]), 'x_init:=' +
                       str(init_pos[batch_num[i]]),
                       'param_ff_torque_1:='+ff_torque_1[i], 'param_ff_torque_2:='+ff_torque_2[i], 'param_time_1:='+time_1[i], 'param_time_2:='+time_2[i]]
        launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
            launch_args)[0], launch_args[2:])]
        launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
        launch.start()
        rospy.loginfo('Gazebo running')

        rospy.sleep(time_init)

        launch_args = ['quad_utils', 'standing.launch']
        launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
            launch_args)[0], launch_args[2:])]
        launch_2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
        launch_2.start()
        rospy.loginfo('Standing')

        rospy.sleep(time_stand)

        launch_args = ['quad_utils', 'planning.launch',
                       'logging:=true', 'tail:=true']
        launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
            launch_args)[0], launch_args[2:])]
        launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
        launch_3.start()
        rospy.loginfo('MPC running')

        rospy.sleep(time_walk)

        launch.shutdown()
        launch_2.shutdown()
        launch_3.shutdown()
