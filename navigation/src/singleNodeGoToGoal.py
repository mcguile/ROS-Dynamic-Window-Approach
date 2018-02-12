#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs import *
from math import atan2
from sensor_msgs.msg import LaserScan

x = 0.0
y= 0.0
theta = 0.0
goal = Point()
obstacle = False

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll,pitch,theta) = euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

def goalCB(msg):
    goal.x = msg.x
    goal.y = msg.y

def laserCB(msg):
    # Find angle of obstacle that is too close
    minDistance = 31 # max laser range is 30
    obsDirection = 0 # angle of obstacle from robot
    global obstacle

    # range of robot laser 180:540 to include a large span but not the extreme
    # edges - 360 is directly in front of the robot
    for angle, distance in enumerate(msg.ranges[200:520]):
        if distance < minDistance:
            minDistance = distance
            obsDirection = angle

    if (minDistance < 0.3):
        obstacle = True
        #print ("obstacle in front")
        # Too close: stop.
        speed.linear.x = 0
        if (obsDirection < 160):
            # Turn right
            speed.angular.z = -0.4
        else:
            # Turn left
            speed.angular.z = 0.4
    else:
        speed.angular.z = 0
        speed.linear.x = 0.3
        if (msg.ranges[0] < 0.3 or msg.ranges[719] < 0.3):
            #print ("obstacle on my side")
            obstacle = True
        else:
            #print ("no obstacle")
            obstacle = False

def setMotion():
    inc_x = goal.x - x
    inc_y = goal.y - y
    angle_to_goal = atan2(inc_y,inc_x)

    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        if ((angle_to_goal - theta) < 0):
            speed.angular.z = -0.1
        else:
            speed.angular.z = 0.1
    else:
        speed.linear.x = 0.3
        speed.angular.z = 0.0

rospy.init_node("goToGoal")
sub = rospy.Subscriber("/odom",Odometry,newOdom)
pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
subGoal = rospy.Subscriber("/goal_pos", Point, goalCB)
subLaser = rospy.Subscriber("/scan", LaserScan, laserCB)

speed = Twist()
r = rospy.Rate(2)

while not rospy.is_shutdown():
    if (obstacle == False):
        setMotion()
    pub.publish(speed)
    r.sleep()
