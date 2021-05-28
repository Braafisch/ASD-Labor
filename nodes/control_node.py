#!/usr/bin/env python3

"""ROS Node for controling vehicle's throttle, brakes and steering.

The task of this node is to move the vehicle along the planned trajectory.
"""

import rospy
from rospy import rostime
import std_msgs.msg
import tf
import numpy as np

from car_demo.msg import Control, Trajectory
from prius_model import Prius_State, L, normalize_angle


class TrajectoryHandler:
    """
    A class to subscribe to trajectory updates and print received data.
    """

    def __init__(self, prius: Prius_State):
        self.k = 0.7
        self.throttle = 0
        self.brake = 0
        self.steer = 0
        self.prius = prius
        self.trajec = None
        last_target_idx = None

        # subscribe to lane coefficient updates
        self.trajectory_sub = rospy.Subscriber(
            "trajectory", Trajectory, self._callback, queue_size=1
        )

    def calc_target_index(self, state, cx, cy, cyaw):
        """
        Compute index in the trajectory list of the target.

        :param state: (State object)
        :param cx: [m] x-coordinates of (sampled) desired trajectory
        :param cy: [m] y-coordinates of (sampled) desired trajectory
        :param cyaw: [rad] tangent angle of (sampled) desired trajectory
        :return: (int, float)
        """
        # Calc front axle position
        fx = state.x + 0.5 * L * np.cos(state.yaw)
        fy = state.y + 0.5 * L * np.sin(state.yaw)

        # Search nearest point index
        dx_vec = fx - np.asarray(cx).reshape([-1, 1])
        dy_vec = fy - np.asarray(cy).reshape([-1, 1])
        dist = np.hstack([dx_vec, dy_vec])
        dist_2 = np.sum(dist ** 2, axis=1)
        target_idx = np.argmin(dist_2)

        # Project RMS error onto front axle vector
        front_axle_vec = [
            np.cos(cyaw[target_idx] + np.pi / 2),
            np.sin(cyaw[target_idx] + np.pi / 2),
        ]
        error_front_axle = np.dot(dist[target_idx, :], front_axle_vec)

        return target_idx, error_front_axle

    def stanley_control(self, state, cx, cy, cyaw, last_target_idx):
        """
        Stanley steering control.

        :param state: (State object)
        :param cx: [m] x-coordinates of (sampled) desired trajectory
        :param cy: [m] y-coordinates of (sampled) desired trajectory
        :param cyaw: [rad] orientation of (sampled) desired trajectory
        :param last_target_idx: [int] last visited point on desired trajectory
        :return: ([rad] steering angle,
            [int] last visited point on desired trajectory,
            [m] cross track error at front axle)
        """
        current_target_idx, error_front_axle = self.calc_target_index(
            state, cx, cy, cyaw
        )

        # make sure that we never match a point on the desired path
        # that we already passed earlier:
        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        ## INSERT CODE HERE
        delta = normalize_angle(cyaw[current_target_idx] - state.yaw) + np.arctan2(
            -self.k * error_front_axle, state.v
        )

        ## END INSERTED CODE

        return delta, current_target_idx, error_front_axle

    def calculate_control(self):
        di, self.last_target_idx, _ = self.stanley_control(
            state=self.prius,
            cx=self.trajec.x,
            cy=self.trajec.y,
            cyaw=self.trajec.theta,
            last_target_idx=self.last_target_idx,
        )

        self.steer = di / np.radians(30.0)
        if self.trajec.v[self.last_target_idx] > self.prius.v:
            self.throttle = 1.0
            self.brake = 0
        else:
            self.brake = 1.0
            self.throttle = 0

    def get_control(self):
        return self.throttle, self.brake, self.steer

    def _callback(self, message: Trajectory):

        rospy.loginfo(
            "trajectory: "
            f"x={str(message.x)}, "
            f"y={str(message.y)}, "
            f"theta={str(message.theta)}, "
            f"c={str(message.c)}, "
            f"v={str(message.v)}, "
            f"s={str(message.s)}"
        )

        self.trajec = message


if __name__ == "__main__":
    rospy.init_node("control_node")
    rate = rospy.Rate(1.0)

    # subscribers
    tf_listener = tf.TransformListener()

    # publishers
    control_pub = rospy.Publisher("prius", Control, queue_size=1)

    # create prius state
    prius = Prius_State()

    # setup lane coefficients handler
    trajectory_handler = TrajectoryHandler(prius)

    # main loop
    time_start = rospy.Time(0)
    while not rospy.is_shutdown():
        time_now = rospy.get_rostime()
        if time_start == rospy.Time(0):
            time_start = time_now

        prius.update_from_joint(time_now)
        trajectory_handler.calculate_control()
        control = Control()
        control.header = std_msgs.msg.Header()
        (
            control.throttle,
            control.brake,
            control.steer,
        ) = trajectory_handler.get_control()
        control.shift_gears = Control.FORWARD

        control_pub.publish(control)

        rate.sleep()
