"""Pose estimate from a position sensor, the gyro and the driven speed.

No node uses this filter for now: the robot has no GNSS receiver, and the
simulation takes the pose from Gazebo (sim_pose_node.py). It stays for when
a position sensor is added.

An extended Kalman filter on the pose (x east, y north, heading). The gyro
yaw rate and the forward speed drive the prediction; each GNSS fix corrects
the position, and through the motion also the heading. The forward speed is
the commanded speed, because the robot has no wheel encoders, or the wheel
odometry where a robot has them.
"""

import math

import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SpeedModel:
    """Forward speed of the robot from the speed its motor driver is told to reach.

    Without wheel encoders the speed is not measured. The driver's lag and the
    acceleration limit of the drive are applied to the commanded speed, so the
    estimate ramps as the robot does.
    """

    def __init__(self, time_constant, max_acceleration):
        self.time_constant = time_constant        # s, lag of the motor driver
        self.max_acceleration = max_acceleration  # m/s^2
        self.lagged = 0.0
        self.speed = 0.0

    def update(self, target, dt):
        """Advance by dt [s] towards the target speed [m/s]. Returns the speed."""
        if dt > 0:
            self.lagged += (target - self.lagged) * (1.0 - math.exp(-dt / self.time_constant))
            step = self.max_acceleration * dt
            self.speed += min(max(self.lagged - self.speed, -step), step)
        return self.speed


class PoseFilter:

    def __init__(self, speed_stddev, yaw_rate_stddev, heading_stddev):
        """speed_stddev [m/s] and yaw_rate_stddev [rad/s] are the errors of the
        prediction inputs; heading_stddev [rad] is the error of the first heading."""
        self.speed_var = speed_stddev ** 2
        self.yaw_rate_var = yaw_rate_stddev ** 2
        self.heading_var = heading_stddev ** 2
        self.state = None
        self.cov = None

    @property
    def ready(self):
        return self.state is not None

    @property
    def pose(self):
        return tuple(float(v) for v in self.state)

    def reset(self, x, y, heading, position_stddev):
        self.state = np.array([x, y, heading], dtype=float)
        self.cov = np.diag([position_stddev ** 2, position_stddev ** 2, self.heading_var])

    def predict(self, v, omega, dt):
        """Advance by forward speed v [m/s] and yaw rate omega [rad/s] over dt [s]."""
        if not self.ready or dt <= 0:
            return
        x, y, th = self.state
        mid = th + omega * dt / 2
        c, s = math.cos(mid), math.sin(mid)
        self.state = np.array([x + v * dt * c, y + v * dt * s, wrap(th + omega * dt)])
        f = np.array([[1, 0, -v * dt * s], [0, 1, v * dt * c], [0, 0, 1]])
        g = np.array([[dt * c, 0], [dt * s, 0], [0, dt]])
        q = np.diag([self.speed_var, self.yaw_rate_var])
        self.cov = f @ self.cov @ f.T + g @ q @ g.T

    def correct(self, x, y, stddev):
        """Fuse a position fix [m] with standard deviation stddev [m]."""
        if not self.ready:
            return
        h = np.array([[1.0, 0, 0], [0, 1.0, 0]])
        s = h @ self.cov @ h.T + np.eye(2) * stddev ** 2
        k = self.cov @ h.T @ np.linalg.inv(s)
        self.state = self.state + k @ (np.array([x, y]) - self.state[:2])
        self.state[2] = wrap(self.state[2])
        self.cov = (np.eye(3) - k @ h) @ self.cov
