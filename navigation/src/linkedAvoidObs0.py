#! /usr/bin/env python
# Basic node to allow avoidance of obstacle. 
# Robot will move linearly until encountering obstacle,
# where it will stop, turn, and move linearly again.
# PHYSICAL BOT CODE
# Angular velocites need to be negated in simulation
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import *
from math import atan2
from sensor_msgs.msg import LaserScan

class Config():
    def __init__(self):
        # Obstacle identifier, passed to a topic
        self.obstacle = False
        self.speed = Twist()
        self.r = rospy.Rate(2)
        self.pubTele = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
        self.pubLaser = rospy.Publisher("/obstacle", Bool, queue_size=10)
        self.subScan = rospy.Subscriber("/scan", LaserScan, self.callback)

    # Callback for scan
    def callback(self,msg):

        minDistance = 31 # max laser range is 30
        obsDirection = 0 # angle of obstacle from robot

        # PHYSICAL BOT HAS DEGREES OF 512
        # SIMULATION HAS DEGREES OF 720
        # Loop without extreme edge vals. They're considered later.
        for angle, distance in enumerate(msg.ranges[1:len(msg.ranges)-2]):
            if distance < minDistance:
                minDistance = distance
                obsDirection = angle

        if (minDistance < 0.3):
            # Obstacle in front: stop.
            self.obstacle = True
            self.speed.linear.x = 0
            # Check which direction to go
            # NOTE PHYSICAL BOT HAS INVERTED LASER DEGREES RIGHT TO LEFT
            # THEREFORE ANGULAR VELS INVERTED
                # Turn right
            if (obsDirection < len(msg.ranges)/2:
                self.speed.angular.z = 1
                # Turn left
            else:
                self.speed.angular.z = -1
        else:
            # No obstacle in front. Move forward
            self.speed.angular.z = 0
            self.speed.linear.x = 0.3
            # Checks to see if obstacle is at side
            if (msg.ranges[0] < 0.5 or msg.ranges[len(msg.ranges)-1] < 0.5):
                self.obstacle = True
            else:
                self.obstacle = False

        self.pubLaser.publish(self.obstacle)

def main():
    config = Config()
    
    while not rospy.is_shutdown():
        if(config.obstacle):
            config.pubTele.publish(config.speed)
        config.r.sleep()
    
if __name__ = '__main__':
    rospy.init_node("obstacles")
    main()
