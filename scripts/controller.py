#!/usr/bin/env python
# coding: utf-8
import rospy
import threading
# import tf.transformations as tftr
from numpy import *

from geometry_msgs.msg import Pose, Point, Vector3
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

lock = threading.Lock()


class Controller:

    def __init__(self):
        self.odometry = None

        # init topics for feedback and control
        self.odom_sub = rospy.Subscriber("/turtlebot/odom", Odometry, self.odometry_callback)
        self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=1)

    def odometry_callback(self, msg):
        """
        Odometry (state)
        Can be used for feedback for trajectory controller
        """
        lock.acquire()
        self.odometry = msg
        lock.release()

    def exists_odometry(self):
        if self.odometry is not None:
            return True
        else:
            return False

    def update(self, dt, pos_des, vel_des):
        """ Update control signal for robotino """
   
        # get feedback vars 
        pos_cur = self.odometry.pose.pose.position

        # compute control error
        x_error = pos_des.linear.x - pos_cur.x
        y_error = pos_des.linear.y - pos_cur.y

        # set control
        velocity = Twist()
        velocity.linear.x = 0.1 * x_error
        velocity.linear.y = 0.1 * y_error
        velocity.angular.z = 0.0
        self.cmd_vel_pub.publish(velocity)


if __name__ == '__main__':

    ctrl = Controller()

    # if ctrl.exists_odometry():
    #     ctrl.update(0.1, ?, ?)
