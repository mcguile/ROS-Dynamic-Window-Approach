#! /usr/bin/env python
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

show_animation = True

# simulation parameters
# robot parameter
max_speed = 0.7  # [m/s]
min_speed = -0.7  # [m/s]
max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
max_accel = 0.2  # [m/ss]
max_dyawrate = 40.0 * math.pi / 180.0  # [rad/ss]
v_reso = 0.01  # [m/s]
yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
dt = 0.1  # [s]
predict_time = 3.0  # [s]
to_goal_cost_gain = 1.0
speed_cost_gain = 1.0
robot_radius = 1.0  # [m]

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

def motion(x, u, dt):
    # motion model

    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x):

    # Dynamic window from robot specification
    Vs = [min_speed, max_speed,
          -max_yawrate, max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - max_accel * dt,
          x[3] + max_accel * dt,
          x[4] - max_dyawrate * dt,
          x[4] + max_dyawrate * dt]
    #  print(Vs, Vd)

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    #  print(dw)

    return dw


def calc_trajectory(xinit, v, y):

    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= predict_time:
        x = motion(x, [v, y], dt)
        traj = np.vstack((traj, x))
        time += dt

    #  print(len(traj))
    return traj


def calc_final_input(x, u, dw, goal, ob):

    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0
    best_traj = np.array([x])

    # evalucate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], v_reso):
        for y in np.arange(dw[2], dw[3], yawrate_reso):
            traj = calc_trajectory(xinit, v, y)

            # calc cost
            to_goal_cost = calc_to_goal_cost(traj, goal)
            speed_cost = speed_cost_gain * \
                (max_speed - traj[-1, 3])
            #ob_cost = calc_obstacle_cost(traj, ob)
            #  print(ob_cost)

            final_cost = to_goal_cost + speed_cost# + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, y]
                best_traj = traj

    #  print(min_u)
    #  input()

    return min_u, best_traj


def calc_obstacle_cost(traj, ob):
    # calc obstacle cost inf: collistion, 0:free

    skip_n = 2
    minr = float("inf")

    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)
            if r <= robot_radius:
                return float("Inf")  # collisiton

            if minr >= r:
                minr = r

    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal):
    # calc to goal cost. It is 2D norm.

    dy = goal[0] - traj[-1, 0]
    dx = goal[1] - traj[-1, 1]
    goal_dis = math.sqrt(dx**2 + dy**2)
    cost = to_goal_cost_gain * goal_dis

    return cost


def dwa_control(x, u, goal, ob):
    # Dynamic Window control

    dw = calc_dynamic_window(x)

    u, traj = calc_final_input(x, u, dw, goal, ob)

    return u, traj

def plot_arrow(x, y, yaw, length=0.5, width=0.1):
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

rospy.init_node("goToGoal")
#sub = rospy.Subscriber("/odom",Odometry,newOdom)
pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
speed = Twist()
r = rospy.Rate(2)
# initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
# goal position [x(m), y(m)]
goal = np.array([10, 10])
# obstacles [x(m) y(m), ....]
ob = np.matrix([[-1, -1],
                [0, 2],
                [4.0, 2.0],
                [5.0, 4.0],
                [5.0, 5.0],
                [5.0, 6.0],
                [5.0, 9.0],
                [8.0, 9.0],
                [7.0, 9.0],
                [12.0, 12.0]
                ])

u = np.array([0.0, 0.0])
traj = np.array(x)

while not rospy.is_shutdown():
    u, ltraj = dwa_control(x, u, goal, ob)
    x = motion(x, u, dt)
    traj = np.vstack((traj, x))  # store state history

    if show_animation:
        plt.cla()
        plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
        plt.plot(x[0], x[1], "xr")
        plt.plot(goal[0], goal[1], "xb")
        plt.plot(ob[:, 0], ob[:, 1], "ok")
        plot_arrow(x[0], x[1], x[2])
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)

    # check goal
    if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= robot_radius:
        print("Goal!!")

    r.sleep()
