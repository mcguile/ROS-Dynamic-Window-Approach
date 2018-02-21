#! /usr/bin/env python
# PHYSICAL BOT CODE
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import *
from math import atan2
from sensor_msgs.msg import LaserScan

# Intialise all velocities,orientation to zero and assume no object
x = 0.0
y= 0.0
theta = 0.0
goal = Point()
obstacle = False

# Method to retrieve odometry of robot
def newOdom(msg):
    global x
    global y
    global theta

    # Attain current position of robot
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # Attain orientation of robot
    rot_q = msg.pose.pose.orientation
    (roll,pitch,theta) = euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

# Callback for attaining goal co-ordinates
def goalCB(msg):
    goal.x = msg.x
    goal.y = msg.y

# Check if obstacle is in the way
# This callback determines which script publishes velocities
# Depending on information published in obstacle topic
def laserCB(msg):
    global obstacle
    # msg is a boolean checking for obstacle
    if(msg.data):
        obstacle = True
    else:
        obstacle = False
# If no obstacle, setMotion will run.
# Based on current co-ordinates and goal co-ords
# Euclidean path will be chosen and pursued providing
def setMotion():
    if (atGoal() is False):
        # Find distances to goal in x & y
        # Use trig to find desired angle to goal
        inc_x = goal.x - x
        inc_y = goal.y - y
        angle_to_goal = atan2(inc_y,inc_x)
        # Check if robot is facing goal
        
        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            # Adjust robot to face goal
            if ((angle_to_goal - theta) < 0):
                speed.angular.z = -1
            else:
                speed.angular.z = 1
                # If facing goal then move towards it
        else:
            speed.linear.x = 0.3
            speed.angular.z = 0.0

def atGoal():
    # Give error margins to stop at goal
    # error above
    maxErx = goal.x + 0.3
    maxEry = goal.y + 0.3
    # error below
    minErx = goal.x - 0.3
    minEry = goal.y - 0.3

    if((minErx < x < maxErx) and (minEry < y < maxEry)):
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        return True
    return False

# Setup subscribers and publishers
rospy.init_node("goToGoal")
sub = rospy.Subscriber("/odom",Odometry,newOdom)
pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
subGoal = rospy.Subscriber("/goal_pos", Point, goalCB)
subLaser = rospy.Subscriber("/obstacle", Bool, laserCB)

speed = Twist()
r = rospy.Rate(2)

while not rospy.is_shutdown():
    # If no obstacle go to goal
    # If obstacle, avoidance node takes over
    if (not obstacle):
        setMotion()
        pub.publish(speed)
    r.sleep()
