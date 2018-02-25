# Minimally modified from https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/1397890f-71c5-4d83-aae7-0a031eedd1f2/concepts/34d4a65f-44d9-462f-b246-c2e653a19c1d

# ----------------
# User Instructions
#
# Implement twiddle as shown in the previous two videos.
# Your accumulated error should be very small!
#
# You don't have to use the exact values as shown in the video
# play around with different values! This quiz isn't graded just see
# how low of an error you can get.
#
# Try to get your error below 1.0e-10 with as few iterations
# as possible (too many iterations will cause a timeout).
#
# No cheating!
# ------------

import random
import numpy as np
import matplotlib.pyplot as plt
import math

# ------------------------------------------------
# 
# this is the Robot class
#

class Robot(object):
    def __init__(self, length=20.0):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
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

############## ADD / MODIFY CODE BELOW ####################
# ------------------------------------------------------------------------
#
# run - does a single control run


def make_robot():
    """
    Resets the robot back to the initial position and drift.
    You'll want to call this after you call `run`.
    """
    robot = Robot()
    robot.set(0, 1, 0)
    robot.set_steering_drift(10 / 180 * np.pi)
    return robot


# NOTE: We use params instead of tau_p, tau_d, tau_i
# def run(robot, params, n=20, speed=1.0):
#     x_trajectory = []
#     y_trajectory = []
#     err = 0
#     prev_cte = robot.y
#     int_cte = 0
#     for i in range(n):
#         cte = robot.y
#         diff_cte = cte - prev_cte
#         int_cte += cte
#         prev_cte = cte
#         steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte
#         robot.move(steer, speed)
#         x_trajectory.append(robot.x)
#         y_trajectory.append(robot.y)

#         err += cte ** 2 + steer ** 2 # <==== From the lesson's original code, removed the `if` condition; i.e. collect squared error from the beginning.
#     return x_trajectory, y_trajectory, err / n

def run(robot, params, total_timesteps=600, speed=1.0):
    tau_p, tau_d, tau_i = params

    x_trajectory = []
    y_trajectory = []
    setpoints = []

    sse = 0

    timesteps_to_change_setpoint = 400
    
    prev_error_value = None
    error_value_integ = 0
    for timestep in range(total_timesteps):
        delta_t = 1

        process_value = robot.y
        setpoint = int(timestep / timesteps_to_change_setpoint) * -1
        # setpoint = 0
        setpoints.append(setpoint)
        error_value = setpoint - process_value

        err_value_derivative = 0 if prev_error_value is None else (error_value - prev_error_value) / delta_t
        prev_error_value = error_value

        error_value_integ += error_value * delta_t

        control_variable = \
            tau_p * error_value + \
            tau_d * err_value_derivative + \
            tau_i * error_value_integ

        robot.move(control_variable, speed * delta_t)

        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)

        # we could normalize `control_variable`
        sse += error_value ** 2 + (control_variable / math.pi) ** 2

    mse = sse / total_timesteps

    return x_trajectory, y_trajectory, setpoints, mse

# Make this tolerance bigger if you are timing out!
def twiddle(tol=0.2): 
    # Don't forget to call `make_robot` before every call of `run`!
    p = [0, 0, 0]
    dp = [1, 1, 1]

    robot = make_robot()
    _, _, _, best_err = run(robot, p)
    
    while sum(dp) > tol:
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            _, _, _, err = run(robot, p)
            if err < best_err:
                best_err = err
                dp[i] *= 1.1

            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                _, _, _, err = run(robot, p)
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1

                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
    
    return p, best_err


params, err = twiddle()
print("params", params)
print("Final twiddle error = {}".format(err))
robot = make_robot()
x_trajectory, y_trajectory, setpoints, err = run(robot, params)
n = len(x_trajectory)

plt.plot(x_trajectory, y_trajectory, 'g', label='Twiddle PID controller')
plt.plot(x_trajectory, setpoints, 'r', label='Setpoints')
plt.savefig('initial-hyperparams.svg')
