#! /usr/bin/env python

import rospy
import threading

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import tf.transformations as tftr
from numpy import matrix, cos, arctan2, sqrt, pi, sin, cos
import numpy as np


class PID:

    def __init__(self, kp, ki, kd, u_max=None, u_min=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.u_max, self.u_min = u_max, u_min
        self.counter = 0
        self.ei = np.matrix(np.zeros((3,1)))
        self.last_e = np.matrix(np.zeros((3,1)))

    def get_control(self, e, dt):

        self.counter += 1
        if self.counter == 1:
            ed = np.matrix(np.zeros((3,1)))
        else:
            self.ei += e * dt
            ed = (e - self.last_e) / dt
        self.last_e = e

        u = np.diag(self.kp, 0).dot(e) + np.diag(self.ki, 0).dot(self.ei) + np.diag(self.kd, 0).dot(ed)

        if self.u_max and self.u_min:
            if u > self.u_max:
                u = self.u_max
            elif u < self.u_min:
                u = self.u_min

        return u

class Task2:

    def __init__(self):
        self.lock = threading.Lock()
        self.RATE = rospy.get_param('/rate', 50)

        self.dt = 0.0
        self.time_start = 0.0
        self.end = False

        self.pose_init = [0.0, 0.0, 0.0]
        self.flag = True

        "Desired values setup"
        # rotation matrix [4x4] from `world` frame to `body`
        self.bTw = tftr.euler_matrix(-np.pi, 0.0, 0.0, 'rxyz')

        # in `world` frame
        self.A = rospy.get_param('/A', 90.0)    # [degrees]
        self.pose_des = rospy.get_param('/pose_des', [0.5, 0.0, 2.0])
             
        # in 'body' frame
        self.pose_des = self.transform_pose(self.pose_des)
        print(self.pose_des.T)

        self.rot_z_des = 0.0

        "Controllers"
        self.pose_controller = PID([1.1, 0.0, 0.0], 
                                   [1.0, 0.0, 0.0],
                                   [1.1, 0.0, 0.0])
        #self.orientation_controller = PID(5.0, 2.0, 0.0)

        "ROS stuff"
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)


    def transform_pose(self, pose_w):
        # in 'body' frame
        pose_des = self.bTw * np.matrix([pose_w[0], pose_w[1], pose_w[2], 0.0]).T
        return pose_des[:3]
            
    def odometry_callback(self, msg):
        self.lock.acquire()
        # read current robot state
        cur_position = msg.pose.pose.position
        cur_q = msg.pose.pose.orientation
        cur_rpy = tftr.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw
        cur_rot_z = cur_rpy[2]

        if self.flag:
            self.zero_pose = [cur_position.x, cur_position.y, cur_position.z]
            self.flag = False


        # errors
        e_x = self.pose_des[0] - cur_position.x
        e_y = self.pose_des[1] - cur_position.y
        error_angle = self.pose_des[1] - cur_rot_z

        # distance to goal
        dist_to_goal = sqrt(e_x**2 + e_y**2)
        if cur_rot_z > pi:
            error_angle = -arctan2(e_y, e_x) + cur_rot_z - 2 * pi
        else:
            error_angle = -arctan2(e_y, e_x) + cur_rot_z

        # set control
        velocity = Twist()
        velocity.linear.x = 0.6 * dist_to_goal * cos(error_angle)
        velocity.angular.z = 1.0 * error_angle
        self.pub_cmd_vel.publish(velocity)

        self.lock.release()

        """
        dist_to_goal = sqrt(e_x**2 + e_y**2)
        if cur_rot_z > pi:
            error_angle = -arctan2(e_y, e_x) + cur_rot_z - 2 * pi
        else:
        error_angle = -arctan2(e_y, e_x) + cur_rot_z
        """
        #print(error_pose.T)
        """
        if not self.end and not self.flag:
            u = self.pose_controller.get_control(error_pose, self.dt)
            print(u.T)   
            velocity.linear.x = u[0]
            velocity.linear.y = u[1]
            velocity.linear.z = u[2] 

        #velocity.linear.y = 0.6 * dist_to_goal * cos(error_angle)
        #velocity.angular.z = -1.0 * error_angle"""
        #self.pub_cmd_vel.publish(velocity)

    def spin(self):
        rospy.loginfo('Task started!')
        rate = rospy.Rate(self.RATE)

        time_step = 5.0
        self.end = False

        time_prev = 0.0
        self.time_start = rospy.get_time()
        while not rospy.is_shutdown():
            t = rospy.get_time() - self.time_start
            self.dt = t - time_prev
            time_prev = t

            self.pose_des = self.transform_pose([5.0, 5.0, 0.0])
            
            #print('time: {:3.3f} dt: {:3.3f}\n'.format(t, self.dt))
            rate.sleep()
        rospy.loginfo('Task completed!')


if __name__ == "__main__":
    rospy.init_node('task2_node')
    task1 = Task2()
    task1.spin()
