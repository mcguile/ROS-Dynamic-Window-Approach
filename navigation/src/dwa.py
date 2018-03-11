#!/usr/bin/env python
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        #NOTE good params:
        #NOTE laser obs 0.2,0,30*pi/180,0.1,30*pi/180,0.01,4*pi/180,0.1,3,0.2,2.0,0.2,0.15
        self.max_speed = 0.2  # [m/s]
        self.min_speed = 0  # [m/s]
        self.max_yawrate = 30.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.1  # [m/ss]
        self.max_dyawrate = 30.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.01  # [m/s]
        self.yawrate_reso = 4.0 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s]
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.2
        self.speed_cost_gain = 2.0
        self.obs_cost_gain = 0.2
        self.robot_radius = 0.15  # [m]
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def assignOdomCoords(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll,pitch,theta) = \
            euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        self.th = theta

class Obstacles():
    def __init__(self):
        self.obst = set()

    def myRange(self,start,end,step):
        i = start
        while i < end:
            yield i
            i += step
        yield end

    def assignObs(self, msg, config):
        deg = len(msg.ranges)
        for angle in self.myRange(0,deg-1,deg/6):
            distance = msg.ranges[angle]
            if (distance < 3):
                # angle of obstacle wrt robot
                scanTheta = (angle/4.0 + deg*(-180.0/deg)+90.0) *math.pi/180.0
                # angle of obstacle wrt global frame
                objTheta = config.th - scanTheta
                # back quadrant negative X negative Y
                if (objTheta < -math.pi):
                    # e.g -405 degrees >> 135 degrees
                    objTheta = objTheta + 1.5*math.pi
                # back quadrant negative X positve Y
                elif (objTheta > math.pi):
                    objTheta = objTheta - 1.5*math.pi

                # round coords to nearest 0.5
                obsX = round((config.x + (distance * math.cos(abs(objTheta))))*4)/4
                # determine direction of Y coord
                if (objTheta < 0):
                    obsY = round((config.y - (distance * math.sin(abs(objTheta))))*4)/4
                else:
                    obsY = round((config.y + (distance * math.sin(abs(objTheta))))*4)/4

                # add coords to set so as to only take unique obstacles
                self.obst.add((obsX,obsY))
                #print self.obst


def motion(x, u, dt):
    # motion model

    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def calc_trajectory(xinit, v, y, config):

    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    return traj


def calc_final_input(x, u, dw, config, goal, ob):

    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for w in np.arange(dw[2], dw[3], config.yawrate_reso):
            traj = calc_trajectory(xinit, v, w, config)

            # calc cost
            to_goal_cost = calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])

            ob_cost = calc_obstacle_cost(traj, ob, config) * config.obs_cost_gain

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, w]
    return min_u


def calc_obstacle_cost(traj, ob, config):
    # calc obstacle cost inf: collision, 0:free

    skip_n = 2
    minr = float("inf")

    for ii in range(0, len(traj[:, 1]), skip_n):
        #for i in range(len(ob[:,0])):
        for i in ob.copy():
            ox = i[0]
            oy = i[1]
            #ox = ob[i,0]
            #oy = ob[i,1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)

            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr


def calc_to_goal_cost(traj, goal, config):
    # calc to goal cost. It is 2D norm.
    if (goal[0] >= 0 and traj[-1,0] < 0):
        dx = goal[0] - traj[-1,0]
    elif (goal[0] < 0 and traj[-1,0] >= 0):
        dx = traj[-1,0] - goal[0]
    else:
        dx = abs(goal[0] - traj[-1,0])

    if (goal[1] >= 0 and traj[-1,1] < 0):
        dy = goal[1] - traj[-1,1]
    elif (goal[1] < 0 and traj[-1,1] >= 0):
        dy = traj[-1,1] - goal[1]
    else:
        dy = abs(goal[1] - traj[-1,1])

    goal_dis = math.sqrt(dx**2 + dy**2)
    cost = config.to_goal_cost_gain * goal_dis
    return cost


def dwa_control(x, u, config, goal, ob):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u = calc_final_input(x, u, dw, config, goal, ob)

    return u


def main():
    print(__file__ + " start!!")
    # robot specification
    config = Config()
    # position of obstacles
    obs = Obstacles()

    subOdom = rospy.Subscriber("/odom", Odometry, config.assignOdomCoords)
    subLaser = rospy.Subscriber("/scan", LaserScan, obs.assignObs, config)
    pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
    speed = Twist()

    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([config.x, config.y, config.th, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([-5,1])
    # initial velocities
    u = np.array([0.0, 0.0])
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        u = dwa_control(x, u, config, goal, obs.obst)
        x[0] = config.x
        x[1] = config.y
        x[2] = config.th
        x[3] = u[0]
        x[4] = u[1]
        speed.linear.x = x[3]
        speed.angular.z = x[4]
        pub.publish(speed)

        # check goal
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) \
            <= config.robot_radius:
            print("Goal!!")
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            break

        r.sleep()
    print("Done")


if __name__ == '__main__':
    rospy.init_node('dwa')
    main()
