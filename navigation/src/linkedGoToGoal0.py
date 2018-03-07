#! /usr/bin/env python
# PHYSICAL BOT CODE
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import *
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
def setMotion(config):
    if (atGoal(config) == False):
        # Find distances to goal in x & y
        # Use trig to find desired angle to goal
        inc_x = config.goal.x - x
        inc_y = config.goal.y - y
        angle_to_goal = atan2(inc_y,inc_x)
        # Check if robot is facing goal
        angle_to_move = config.theta - angle_to_goal

        #print(angle_to_move)

        if(angle_to_move < (-1 * 3.1416)):
            angle_to_move = angle_to_move + (2*3.1416)
        if(angle_to_move > 3.1416):
            angle_to_move = angle_to_move - (2*3.141)

        #print(angle_to_move)

        if (abs(angle_to_move) > 0.1):
            config.speed.linear.x = 0.0
            # Adjust robot to face goal
            if (angle_to_move < 0):
                config.speed.angular.z = 1
            else:
                config.speed.angular.z = -1
                # If facing goal then move towards it
        else:
            config.speed.linear.x = 0.3
            config.speed.angular.z = 0.0

def atGoal(config):
    # Give error margins to stop at goal
    # error above
    maxErx = config.goal.x + 0.15
    maxEry = config.goal.y + 0.15
    # error below
    minErx = config.goal.x - 0.15
    minEry = config.goal.y - 0.15

    if((minErx < config.x < maxErx) and (minEry < config.y < maxEry)):
        config.speed.linear.x = 0.0
        config.speed.angular.z = 0.0
        return True     # GOAL
    return False


def main():
    config = Config()
    while not rospy.is_shutdown():
        # If no obstacle go to goal
        # If obstacle, avoidance node takes over
        if (not config.obstacle):
            setMotion(config)
            config.pub.publish(config.speed)
        config.r.sleep()
    
if __name__ = '__main__':
    rospy.init_node("goToGoal")
    main()
