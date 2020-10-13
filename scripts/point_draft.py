#! /usr/bin/env python

import rospy
import threading
import tf.transformations as tftr
import numpy as np


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0.0
y = 0.0
theta = 0.0

class PID:

	def __init__(self, kp, ki, kd, u_max=None, u_min=None):
		self.kp, self.ki, self.kd = kp, ki, kd
		self.u_max, self.u_min = u_max, u_min
		self.counter = 0
		self.ei = np.zeros([0.0, 0.0, 0.0]).T

	def get_control(self, e, dt):

		self.counter += 1
		if self.counter == 1:
			ed = 0.0
		else:
			self.ei += e*dt
			ed = (e - self.last_e) / dt
		self.last_e = e

		u = self.kp * e + self.ki * self.ei + self.kd * ed
		
		if self.u_max and self.u_min:
			if u > self.u_max:
				u = self.u_max
			elif u < self.u_min
				u = self.u_min

		return u

class Point:

	def __init__(self):
		self.lock = threading.Lock()
		self.RATE = rospy.get_param('/rate', 50)

		self.dt = 0.0
		self.end = False
