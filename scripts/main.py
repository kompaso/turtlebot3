#!/usr/bin/env python
# coding: utf-8
import rospy

from traj_generator import TrajectoryGenerator
from controller import Controller


if __name__ == '__main__':

    # 1. Create new node
    rospy.init_node("main_node")

    rate = rospy.Rate(5)   # controller freq in Hz
    
    # init our trajectory generator and controller
    traj_generator_node = TrajectoryGenerator()
    ctrl = Controller()

    # run main loop
    time_prev = 0
    time_start = rospy.get_time()
    while not rospy.is_shutdown():
        # compute current time and dt
        t = rospy.get_time() - time_start
        dt = t - time_prev
        time_prev = t

        if ctrl.exists_odometry(): # if robot exists
            pose, vel = traj_generator_node.get_point(t)    # get trajectory point for current time
            ctrl.update(dt, pose, vel)                      # send command to robot
            print(t, "Robot is moving!")
        else:
            print("No robot!")

        rate.sleep()                                        # wait to obtain loop frequency
