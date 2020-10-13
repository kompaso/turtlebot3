#!/usr/bin/env python
# coding: utf-8
from numpy import sqrt, sin, cos, arctan2, pi, trunc, finfo, float32, arange

from geometry_msgs.msg import Pose2D, Twist, Vector3


class TrajectoryGenerator:

    def __init__(self):

        # trajectory parameters (initial)
        l = 2 * pi                      # length of path (i dont know how it compute, so just const)
        velocity = 0.1                  # desired velocity for a lap

        self.A_x, self.A_y = 1., 1.     # wide*2 of the trajectory along x and y
        self.phi = -pi / 2.             # offset for 8-shape or more complex curves


        self.tf = l / velocity          # time for lap
        self.omega_x = 2 * pi / self.tf
        self.omega_y = 2 * self.omega_x     # frequencies for 8-like-trajectory

        self.old_xi, self.old_yi = 0., 0.

    def get_point(self, t):
        """
            Возвращает Положение, ориентацию и скорости в момент времени t        
        """

        # compute trajectory point and orientation
        xi = self.A_x * cos(self.omega_x * t + self.phi)
        yi = self.A_y * sin(self.omega_y * t)
        theta = arctan2(yi - self.old_yi, xi - self.old_xi)
        self.old_xi, self.old_yi = xi, yi

        # compute the derivatives of trajectory point and orientation
        dot_xi = -self.A_x * self.omega_x * sin(self.omega_x * t + self.phi)
        dot_yi = self.A_y * self.omega_y * cos(self.omega_y * t)

        if abs(xi) < finfo(float).eps:
            xi = xi + finfo(float).eps
        if (yi / xi)**2 == 1.:
            yi = yi + finfo(float).eps
        dot_theta = 1. / (1 + (yi / xi)**2)
        
        return Pose2D(xi, yi, theta), Twist(Vector3(dot_xi, dot_yi, 0.), Vector3(0., 0., dot_theta))

if __name__ == '__main__':
    # 1) create new object of traj generator
    traj_generator_node = TrajectoryGenerator()

    # 2) get the point for specific time
    t = 0.5
    pose, vel = traj_generator_node.get_point(t)

    print(pose, vel)


    # 3) test it for more times. For instance inside controller

    for t in arange(0, 1, .1):
        pose, vel = traj_generator_node.get_point(t)
        print(t, pose.x, pose.y, pose.theta)