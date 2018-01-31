#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *

def callback(msg):
    moveForward(msg)

def setGoalAndAngle(x,y):
    gradient = (x/y)
    angle = gradient/4 # 90-deg = 0.5
    #todo

def moveForward(msg):
    print msg.pose.pose.position
    if (twist.linear.x == 0):
        twist.angular.z = 0
    if (msg.pose.pose.position.x < 1.5):
        twist.linear.x = 0.1
    else:
        twist.linear.x = 0

rospy.init_node('moveOdometry')
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
sub = rospy.Subscriber('odom',Odometry,callback)
rate = rospy.Rate(2) # refresh rate of messages
twist = Twist() # message that takes the linear and angular velocties of the robot
setGoalAndAngle(1.5,1.5)
# Continually loop while robot is launched
while not rospy.is_shutdown():
    pub.publish(twist) # publish current robot velocities
    rate.sleep() # allow execution of instructions
