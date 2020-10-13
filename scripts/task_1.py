#!/usr/bin/env python
# coding: utf-8
"""
Задача 1. Взлет, маневр, посадка.
 
Время на выполнение: 120 минут. 
Оборудование:   DJI Tello
Сложность: 5%

Требуется реализовать ROS-ноду, автономно выполняющую следующую последовательность действий:
1) Взлететь на заданную высоту H метров
2) Изменить ориентацию квадрокоптера вокруг оси Z в связанной с квадрокоптером системе координат на заданные угол A (в градусах)
3) Изменить высоту на h
4) Выполнить посадку

Входные данные
Три вещественных ROS-параметра  /H, /A, /h.

"""
import rospy
import threading
import tf.transformations as tftr
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8
from nav_msgs.msg import Odometry
from tello_driver.msg import TelloStatus


class PID:

    def __init__(self, kp, ki, kd, u_max=None, u_min=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.u_max, self.u_min = u_max, u_min
        self.counter = 0
        self.ei = np.zeros([0.0, 0.0, 0.0]).T
        self.last_e = 0.0

    def get_control(self, e, dt):

        self.counter += 1
        if self.counter == 1:
            ed = 0.0
        else:
            self.ei += e * dt
            ed = (e - self.last_e) / dt
        self.last_e = e

        u = self.kp * e + self.ki * self.ei + self.kd * ed

        if self.u_max and self.u_min:
            if u > self.u_max:
                u = self.u_max
            elif u < self.u_min:
                u = self.u_min

        return u


class Task1:
    
    def __init__(self):
        self.lock = threading.Lock()
        self.RATE = rospy.get_param('/rate', 50)
        
        self.dt = 0.0
        self.time_start = 0.0
        self.end = False

        "Desired values setup"
        # rotation matrix [4x4] from `world` frame to `body`
        bTw = tftr.euler_matrix(-np.pi, 0.0, 0.0, 'rxyz')

        # in `world` frame
        self.H = rospy.get_param('/H', 1.0)     # [m]
        self.A = rospy.get_param('/A', 90.0)    # [degrees]
        self.h = rospy.get_param('/h', -0.5)    # [m]
             
        # in 'body' frame
        Hb = bTw * np.matrix([0.0, 0.0, self.H, 0.0]).T
        self.z_des = Hb[2]  # desired altitude
        self.rot_z_des = 0.0 # transform rotation around z-axis
        
        "Controllers"
        self.altitude_controller = PID(4.0, 0.0, 1.5)
        self.orientation_controller = PID(5.0, 2.0, 0.0)

        "ROS stuff"
        self.pub_takeoff = rospy.Publisher("/tello/takeoff", Empty, queue_size=1, latch=False)
        self.pub_land = rospy.Publisher("/tello/land", Empty, queue_size=1, latch=False)
        self.pub_cmd_vel = rospy.Publisher("/tello/cmd_vel", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber("/tello/odom", Odometry, self.odometry_callback)
        
    def odometry_callback(self, msg):
        self.lock.acquire()
        
        "drone state"
        cur_position = msg.pose.pose.position
        cur_q = msg.pose.pose.orientation
        cur_rpy = tftr.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw

        "errors"
        # Altitude controller in `body`
        error_z = self.z_des - cur_position.z

        # Orientation controller in `body`
        error_rot_z = self.rot_z_des - cur_rpy[2]
            
        if not self.end:
            "set control"
            velocity = Twist()
            velocity.linear.z = self.altitude_controller.get_control(error_z, self.dt)
            velocity.angular.z = self.orientation_controller.get_control(error_rot_z, self.dt)
            self.pub_cmd_vel.publish(velocity)

        print("{} {}".format(error_z, error_rot_z))
        self.lock.release()    
        
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
            
            if t < 1. * time_step:      # wait while drone takeoff
                self.pub_takeoff.publish(Empty())
                rospy.loginfo('takeoff')
            elif t < 2. * time_step:      # wait while drone takeoff
                #self.z_des = -1.5
                self.rot_z_des = -np.deg2rad(self.A)
            elif t < 3. * time_step:      # wait while drone takeoff
                self.z_des = -1.5
                #self.rot_z_des = -np.deg2rad(-90.0)
            elif t < 4. * time_step:    # wait while drone land
                self.z_des = -1.0
                self.rot_z_des = -np.deg2rad(0.0)
            elif t < 5. * time_step:    # wait while drone land
                self.end = True
                self.pub_land.publish(Empty())
                rospy.loginfo('land')
            elif t < 6. * time_step:    # close the node
                rospy.loginfo('break')
                break
            print('time: {:3.3f} dt: {:3.3f}\n'.format(t, self.dt))
            rate.sleep()
        rospy.loginfo('Task completed!')

if __name__ == "__main__":
    rospy.init_node('task1_node')
    task1 = Task1()
    task1.spin()
    
    
    
    

