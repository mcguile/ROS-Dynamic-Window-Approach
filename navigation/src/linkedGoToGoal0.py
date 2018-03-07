#! /usr/bin/env python
# PHYSICAL BOT CODE
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import *
import math
from math import atan2
from sensor_msgs.msg import LaserScan

class Config():

    def __init__(self):
        # Setup subscribers and publishers
        self.sub = rospy.Subscriber("/odom",Odometry,self.odomCB)
        self.pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
        self.subGoal = rospy.Subscriber("/goal_pos", Point, self.goalCB)
        self.subLaser = rospy.Subscriber("/obstacle", Bool, self.laserCB)
        self.speed = Twist()     # to be published as teleop
        self.r = rospy.Rate(2)   # refresh rate of main loop
        # Intialise all velocities,orientation to zero and assume no object
        self.x = 0.0
        self.y= 0.0
        self.theta = 0.0
        self.goal = Point()
        self.obstacle = False
        self.r = rospy.Rate(2)   # refresh rate of main loop

    # Method to retrieve odometry of robot
    def odomCB(self,msg):
        # Attain current position of robot
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Attain orientation of robot
        rot_q = msg.pose.pose.orientation
        (roll,pitch,self.theta) = euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

    # Callback for attaining goal co-ordinates
    def goalCB(self,msg):
        self.goal.x = msg.x
        self.goal.y = msg.y

    # Check if obstacle is in the way
    # This callback determines which script publishes velocities
    # Depending on information published in obstacle topic
    def laserCB(self,msg):
        # msg is a boolean checking for obstacle
        if(msg.data):
            self.obstacle = True
        else:
            self.obstacle = False

# If no obstacle, setMotion will run.
# Based on current co-ordinates and goal co-ords
# Euclidean path will be chosen and pursued providing
    def setMotion(self):
        # Find distances to goal in x & y
        # Use trig to find desired angle to goal
        inc_x = self.goal.x - self.x
        inc_y = self.goal.y - self.y
        angle_to_goal = atan2(inc_y,inc_x)
        # Check if robot is facing goal
        angle_to_move = self.theta - angle_to_goal

        if(angle_to_move < (-math.pi)):
            angle_to_move = angle_to_move + (2*math.pi)
        elif(angle_to_move > math.pi):
            angle_to_move = angle_to_move - (2*math.pi)

        if (abs(angle_to_move) > 0.1):
            self.speed.linear.x = 0.0
            # Adjust robot to face goal
            if (angle_to_move < 0):
                self.speed.angular.z = 0.3
            else:
                self.speed.angular.z = -0.3
        # If facing goal then move towards it
        else:
            self.speed.linear.x = 0.3
            self.speed.angular.z = 0.0

    def atGoal(self):
        # Give error margins to stop at goal
        # error above
        maxErx = self.goal.x + 0.15
        maxEry = self.goal.y + 0.15
        # error below
        minErx = self.goal.x - 0.15
        minEry = self.goal.y - 0.15

        if((minErx < self.x < maxErx) and (minEry < self.y < maxEry)):
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
            return True     # GOAL
        return False


def main():
    config = Config()
    while not rospy.is_shutdown():
        # If no obstacle go to goal
        # If obstacle, avoidance node takes over
        if (not config.obstacle):
            if (config.atGoal() == False)
                config.setMotion()
            else:
                print ("goal!")
                break
            config.pub.publish(config.speed)
        config.r.sleep()

if __name__ == '__main__':
    rospy.init_node("goToGoal")
    main()
