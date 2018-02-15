#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
from sensor_msgs.msg import LaserScan
import numpy as np

obstacles = np.empty([720,2])
currXYtheta = [0,0,0]

def callbackOdom(msg):
    global currXYtheta
    currXYtheta[0] = msg.pose.position.x
    currXYtheta[1] = msg.pose.position.y
    # Attain orientation of robot
    rot_q = msg.pose.pose.orientation
    (roll,pitch,theta) = euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
    currXYtheta[2] = theta

def callbackLaser(msg):
    global obstacles
    for angle, distance in enumerate(msg.ranges[:]):
        #TODO

def calc_obstacle_cost(traj,config):
    # calc obstactle cos; inf = collision, 0 = free

    skip_n = 2
    minr = float("inf")
    
    # traj is 2D array (vstack) [trajectories, current&final state]
    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(obstacles[:, 0])):
            ox = obstacles[i, 0]
            oy = obstacles[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)
            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr

rospy.init_node("dwa")
subScan = rospy.Subscriber("/scan", LaserScan, callbackLaser)
subOdom = rospy.Subscriber("/odom", Odometry, callbackOdom)

r = rospy.Rate(2)

while not rospy.is_shutdown():
    r.sleep()
