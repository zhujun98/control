# -----------
#
# Basic PID controller:

# steering = -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
#
# where CTE stands for cross-track error.
#
#
# "twiddle" algorithm
#
# https://www.youtube.com/watch?v=2uQ2BSzDvXs
#
# is also implemented here.
#
# ------------

import random
import numpy as np
import matplotlib.pyplot as plt


class Robot(object):
    def __init__(self, length=20.0):
        """Initialization"""
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """Set a robot coordinate"""
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """Set the noise parameters"""
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """Set the systematical steering drift parameter"""
        self.steering_drift = drift

    def move(self, steering, distance, tolerance=0.001,
             max_steering_angle=np.pi/4.0):
        """Move the robot

        :param steering: float
            Front wheel steering angle, limited by max_steering_angle
        :param distance: float
            Total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)


def make_robot():
    """Create a robot"""
    robot_ = Robot()
    robot_.set(0, 1, 0)
    robot_.set_steering_drift(10*np.pi/180)

    return robot_


def run_pid(robot_, params, n=100, speed=1.0):
    """PID controller"""
    tau_d = 0.0
    tau_i = 0.0
    if not isinstance(params, (list, tuple)):
        params = [params]

    tau_p = params[0]
    if len(params) > 1:
        tau_d = params[1]

    if len(params) > 2:
        tau_i = params[2]

    x_trajectory_ = []
    y_trajectory_ = []

    err = 0.0
    cte_sum = 0.0
    cte_old = robot_.y
    for i in range(2*n):  # Note the range is 2n!
        cte = robot_.y

        cte_sum += cte
        delta_cte = cte - cte_old
        cte_old = cte

        steer = -tau_p * cte - tau_d * delta_cte - tau_i*cte_sum

        robot_.move(steer, speed)

        x_trajectory_.append(robot_.x)
        y_trajectory_.append(robot_.y)

        if i >= n:  # Give the algorithm n points to converge!
            err += cte**2

    return x_trajectory_, y_trajectory_, err/n


def twiddle(tol=0.1):
    """Twiddle - CS373 Unit 5 - Udacity"""
    p = [0, 0, 0]
    dp = [1, 1, 1]  # potential changes

    robot_ = make_robot()
    _, _, best_err = run_pid(robot_, p)  # best error

    while sum(dp) > tol:
        for i in range(len(dp)):
            p[i] += dp[i]  # try one direction

            robot_ = make_robot()
            _, _, err = run_pid(robot_, p)  # get the error

            if err < best_err:  # there is improvement
                best_err = err
                dp[i] *= 1.1  # increase the change step
            else:
                p[i] -= 2*dp[i]  # go to other direction

                robot_ = make_robot()
                _, _, err = run_pid(robot_, p)  # get the error

                if err < best_err:  # there is improvement
                    best_err = err
                    dp[i] *= 1.1  # increase the change step
                else:
                    p[i] += dp[i]  # fail at both directions? go back
                    dp[i] *= 0.9  # decrease the change step

    return p


fig, ax = plt.subplots()

robot = make_robot()
x_trajectory, y_trajectory, _ = run_pid(robot, [0.2])
ax.plot(x_trajectory, y_trajectory, 'k', label='P controller')

robot = make_robot()
x_trajectory, y_trajectory, _ = run_pid(robot, [0.2, 3.0])
ax.plot(x_trajectory, y_trajectory, 'orange', label='PD controller')

robot = make_robot()
x_trajectory, y_trajectory, _ = run_pid(robot, [0.2, 3.0, 0.004])
ax.plot(x_trajectory, y_trajectory, 'purple', label='PID controller')

params_opt = twiddle(0.0001)
robot = make_robot()
x_trajectory, y_trajectory, _ = run_pid(robot, params_opt)
ax.plot(x_trajectory, y_trajectory, 'blue', label='PID controller (twiddle)')

ax.plot(x_trajectory, np.zeros(len(x_trajectory)), 'r', label='reference')

ax.legend()

plt.show()
