#! /usr/bin/env python

# basic obstacle avoidance - not used in DWA.

import rospy, math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

#initiate node
rospy.init_node('robot_cleaner')

rate = rospy.Rate(2)

x = 0
y = 0

def mapScanToOdom(value, scanMin, scanMax, odomLeft, odomRight):

    #Find span between ranges
    scanRange = abs(scanMax - scanMin) #720
    odomRange = abs(odomRight - odomLeft) #180

    #Calculate mapped value
    returnTheta = (value - scanMin)*(odomRange/float(scanRange)) + abs(odomLeft)

    #print(returnTheta)
    return returnTheta

def odomCallback(msg):

    global x
    global y

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

def scanCallback(msg):

    for angle, distance in enumerate(msg.ranges[:]):
        if (distance < 5):

            ang = math.radians(mapScanToOdom(angle, 0, 720, 0, 180))

            obsX = x + (distance * abs(math.sin(ang)))
            obsY = y + (distance * abs(math.cos(ang)))

            print(obsX, obsY)

        else:
            obsY = 0
            obsX = 0




subOdom = rospy.Subscriber('/odom', Odometry, odomCallback)
subLaser = rospy.Subscriber('/scan', LaserScan, scanCallback)
 #allows time for exectuion of actions


while not rospy.is_shutdown():

    rate.sleep()
