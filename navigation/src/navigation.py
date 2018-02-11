#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import *
from math import atan2
from sensor_msgs.msg import LaserScan

obstacle = False

#Callback for scan
def callback(msg):

    minDistance = 31 # max laser range is 30
    obsDirection = 0 # angle of obstacle from robot
    global obstacle

    # range of robot laser 180:540 to include a large span but not the extreme
    # edges - 360 is directly in front of the robot
    for angle, distance in enumerate(msg.ranges[260:460]):
        if distance < minDistance:
            minDistance = distance
            obsDirection = angle

    if (minDistance < 0.3):
        obstacle = True
        #print ("obstacle in front")
        # Too close: stop.
        speed.linear.x = 0
        #Checks if obstacle infront of robot and which direction to go
        #enum checks between 260 -> 460, 100 is 360 between these
        if (obsDirection < 100):
            # Turn right
            speed.angular.z = -1.0
        else:
            # Turn left
            speed.angular.z = 1.0
    else:
        #Move forward as obstacle is at the side
        #or no obstacle
        speed.angular.z = 0
        speed.linear.x = 0.3
        #Checks to see if obstacle is at side
        if (msg.ranges[0] < 0.5 or msg.ranges[719] < 0.5):
            obstacle = True
        else:
            obstacle = False

    pubLaser.publish(obstacle)

rospy.init_node("robot_cleaner")
pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
pubLaser = rospy.Publisher("/obstacle", Bool, queue_size=10)
subScan = rospy.Subscriber("/scan", LaserScan, callback)

speed = Twist()
r = rospy.Rate(2)

while not rospy.is_shutdown():
    if(obstacle == True):
        pub.publish(speed)
    r.sleep()
