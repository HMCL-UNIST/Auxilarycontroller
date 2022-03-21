#!/usr/bin/env python3
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
base class for spawning a Ego Vehicle in ROS

Two modes are available:
- spawn at random Carla Spawnpoint
- spawn at the pose read from ROS topic /initialpose

Whenever a pose is received via /initialpose, the vehicle gets respawned at that
position. If no /initialpose is set at startup, a random spawnpoint is used.

/initialpose might be published via RVIZ '2D Pose Estimate" button.
"""

from abc import abstractmethod
from distutils.spawn import spawn
# from cmath import inf


import os
from pdb import Restart
import sys
import random
import math
import json
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped, Vector3, TwistStamped
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int64
from autoware_msgs.msg import Lane, Waypoint
# from autoware_msgs.msg import Lane, Waypoint
import threading
import glob
import numpy as np
import time
import argparse
import logging
import scipy.linalg as la
import easydict
# import resetworld
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

secure_random = random.SystemRandom()

def set_high_priority():
  """ Change process priority to the highest possible. """
  # use "real time" scheduler
  done = False
  sched = os.SCHED_FIFO
  sched_priority = os.sched_get_priority_max(sched)
  param = os.sched_param(sched_priority)
  try:
    os.sched_setscheduler(0, sched, param)
  except OSError:
    rospy.logwarn("Failed to set real time process scheduler to %u, priority %u" % (sched, sched_priority))
  else:
    done = True
    rospy.loginfo("Process real time scheduler set to %u, priority %u" % (sched, sched_priority))
    # rospy.loginfo("Main Thread: %s", threading.current_thread())
  if not done:
    # renice to highest priority
    target_niceness = -19
    previous_niceness = os.nice(0)
    delta_niceness = target_niceness - previous_niceness
    try:
      new_niceness = os.nice(delta_niceness)
    except OSError:
      new_niceness = previous_niceness
    if new_niceness != target_niceness:
      rospy.logwarn("Unable to renice process to %d, current niceness is %d" % (target_niceness, new_niceness))
    else:
      rospy.loginfo("Process reniced from %d to %d" % (previous_niceness, new_niceness))


k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time difference
L = 2.9  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle

class waypointManage(object):
    def __init__(self):
        rospy.init_node('secondary_controller_stanley', anonymous=True)
        self.position = []
        self.wps = []
        self.target = Vector3()
        self.gnssPoseRelay = Vector3()
        self.ackermanCmd = AckermannDrive()
        self.currVel = 0.0
        self.pub1 = rospy.Publisher('/nearest_waypoint', Vector3, queue_size=1)
        self.pub2 = rospy.Publisher('/gnss_pose_relay', Vector3, queue_size=1)
        # self.pub3 = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd1', AckermannDrive, queue_size=1)

    def currPoseCallback(self, data):
        x = round(data.pose.position.x, 2)
        y = round(data.pose.position.y, 2)
        quaternion = (data.pose.orientation.x,
                        data.pose.orientation.y,
                        data.pose.orientation.z,
                        data.pose.orientation.w)

        euler = euler_from_quaternion(quaternion)
        yaw = round(math.degrees(euler[2]), 2)
        self.position = np.array([x, y, yaw])
        self.gnssPoseRelay.x = x
        self.gnssPoseRelay.y = y
        self.gnssPoseRelay.z = yaw
        # rospy.loginfo('here1')

    def waypointsCallback(self, data):
        
        waypoints = data.waypoints
        wps = []
        for waypoint in waypoints:
            x = round(waypoint.pose.pose.position.x, 2)
            y = round(waypoint.pose.pose.position.y, 2)
            quaternion = (waypoint.pose.pose.orientation.x,
                        waypoint.pose.pose.orientation.y,
                        waypoint.pose.pose.orientation.z,
                        waypoint.pose.pose.orientation.w)
            euler = euler_from_quaternion(quaternion)
            yaw = round(math.degrees(euler[2]), 2)
            wps = [x, y, yaw]
            self.wps.append(wps)

    def currVelCallback(self, data):
        self.currVel = data.twist.linear.x

    def callback(self, event=None):
        rospy.loginfo("Subscribing state is Ok...")

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def pid_control(self, target, current):
        """
        Proportional control for the speed.
        :param target: (float)
        :param current: (float)
        :return: (float)
        """
        return Kp * (target - current)

    def calc_nearest_index(self):
        wpslist = np.array(self.wps)
        # rospy.loginfo(wpslist)
        # for icx in wpslist:
        #     rospy.loginfo(icx)
        dx = [self.position[0] - icx[0] for icx in wpslist]
        dy = [self.position[1] - icx[1] for icx in wpslist]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
        # rospy.loginfo(d)
        mind = min(d)

        ind = d.index(mind)

        mind = math.sqrt(mind)

        dxl = wpslist[ind][0] - self.position[0]
        dyl = wpslist[ind][1] - self.position[1]

        # angle = self.pi_2_pi(self.wps[ind][2] - math.atan2(dyl, dxl))
        # if angle < 0:
        #     mind *= -1

        self.target.x = wpslist[ind][0]
        self.target.y = wpslist[ind][1]
        self.target.z = wpslist[ind][2]

    def calc_target_index(self):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        wpslist = np.array(self.wps)
        # Calc front axle position
        fx = self.position[0] + L * np.cos(math.radians(self.position[2]))
        fy = self.position[1] + L * np.cos(math.radians(self.position[2]))
        # Search nearest point index
        dx = [self.position[0] - icx[0] for icx in wpslist]
        dy = [self.position[1] - icx[1] for icx in wpslist]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(math.radians(self.position[2]) + np.pi / 2),
                        -np.sin(math.radians(self.position[2])+ np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle

    def stanley_control(self, last_target_idx):
        """
        Stanley steering control.
        :param state: (State object)
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        wpslist = np.array(self.wps)
        current_target_idx, error_front_axle = self.calc_target_index()

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = self.normalize_angle(wpslist[current_target_idx][2] - math.radians(self.position[2]))
        # theta_d corrects the cross track error
        theta_d = np.arctan2(k * error_front_axle, self.currVel)
        # Steering control
        delta = theta_e + theta_d

        return delta, current_target_idx
        
    def publisher(self,event=None):
        if (len(self.wps) >= 1) and (len(self.position) >= 1):
            j = 0
            if j == 0:
                target_idx, _ = self.calc_target_index()
                j = 1
            self.calc_nearest_index()
            target_speed = 60.0 / 3.6  # [m/s]
            ai = self.pid_control(target_speed, self.currVel)
            di, target_idx = self.stanley_control(target_idx)
            
            self.ackermanCmd.steering_angle = di
            self.ackermanCmd.steering_angle_velocity = 0
            self.ackermanCmd.speed = 0
            self.ackermanCmd.acceleration = ai
            self.ackermanCmd.jerk = 0

            self.pub1.publish(self.target)
            self.pub2.publish(self.gnssPoseRelay)
            # self.pub3.publish(self.ackermanCmd)
            rospy.loginfo("Relay start...\n")
        else:
            rospy.loginfo("Not yet initialized...")

    def run(self): # main loop
        # Subscribe topic at 100Hz
        rospy.Subscriber("/final_waypoints", Lane, self.waypointsCallback)
        rospy.Subscriber("/gnss_pose", PoseStamped, self.currPoseCallback)
        rospy.Subscriber("/current_velocity", TwistStamped, self.currVelCallback)
        # rospy.Timer(rospy.Duration(1.0/100), self.callback)
        # Pub control cmd at 50 Hz
        rospy.Timer(rospy.Duration(1.0/50), self.publisher)
        
        rospy.spin()
        

        
# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

if __name__ == '__main__': 
    try:  
        # set_high_priority() 
        wm_ = waypointManage()            
        wm_.run()  
        
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
