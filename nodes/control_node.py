#!/usr/bin/env python3

"""ROS Node for controling vehicle's throttle, brakes and steering.

The task of this node is to move the vehicle along the planned trajectory.
"""

import rospy
import std_msgs.msg
import tf
import numpy as np

from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

from car_demo.msg import Control, Trajectory
from prius_model import Prius_State, L, normalize_angle, veh_dim_x, veh_dim_y, max_steer


class TrajectoryHandler:
    """
    A class to subscribe to trajectory updates and print received data.
    """

    def __init__(self, prius: Prius_State):
        self.k = 0.7
        self.throttle = 0
        self.brake = 0
        self.steer = 0
        self.gear = Control.NO_COMMAND
        self.prius = prius
        self.trajec = None
        self.last_target_idx = None

        # subscribe to lane coefficient updates
        self.trajectory_sub = rospy.Subscriber(
            "trajectory", Trajectory, self._callback, queue_size=1
        )

        self.control_dbg_pub = rospy.Publisher("control_dbg", Marker, queue_size=1)

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
        if last_target_idx is not None and last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        delta = normalize_angle(cyaw[current_target_idx] - state.yaw) + np.arctan2(
            -self.k * error_front_axle, state.v
        )

        return delta, current_target_idx, error_front_axle

    def calculate_control(self):
        if self.trajec is not None:
            di, self.last_target_idx, _ = self.stanley_control(
                state=self.prius,
                cx=self.trajec.x,
                cy=self.trajec.y,
                cyaw=self.trajec.theta,
                last_target_idx=self.last_target_idx,
            )
            marker = create_debug_marker(
                101,
                self.trajec.x[self.last_target_idx],
                self.trajec.y[self.last_target_idx],
            )
            self.control_dbg_pub.publish(marker)
            ai = self.trajec.v[self.last_target_idx] - self.prius.v
            di = np.clip(di, -max_steer, max_steer)
            self.steer = di / np.radians(30.0)
            if ai >= 0:
                self.throttle = 0.01
                self.brake = 0
            else:
                self.brake = 0.5
                self.throttle = 0

    def get_control(self):
        return self.throttle, self.brake, self.steer, self.gear

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
        self.last_target_idx = 0
        self.gear = Control.FORWARD


def create_debug_marker(id: int, x: float, y: float, size: float = 5) -> Marker:
    marker = Marker()
    marker.header = std_msgs.msg.Header()
    marker.header.frame_id = "map"
    marker.id = id
    marker.action = Marker.ADD
    marker.type = Marker.CUBE
    marker.pose.position = Point(x=x, y=y)
    marker.color = ColorRGBA(r=1, g=1, b=0, a=1)
    marker.scale = Vector3(x=size * 0.1, y=size * 0.1, z=size * 0.1)
    return marker


if __name__ == "__main__":
    rospy.init_node("control_node")
    rate = rospy.Rate(50.0)

    # subscribers
    tf_listener = tf.TransformListener()

    # publishers
    control_pub = rospy.Publisher("prius", Control, queue_size=1)

    # create prius state
    prius = Prius_State(x=veh_dim_x, y=veh_dim_y)

    # setup lane coefficients handler
    trajectory_handler = TrajectoryHandler(prius)

    # main loop
    while not rospy.is_shutdown():
        time_now = rospy.get_rostime()
        if time_start == rospy.Time(0):
            time_start = time_now
        try:
            prius.update(time_now)
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.loginfo("Could not receive information!")
            try:
                rate.sleep()
            except rospy.exceptions.ROSInterruptException as e:
                rospy.logdebug(
                    f"Stopping {rospy.get_name()}, because of interrupt: {e}"
                )
            continue
        trajectory_handler.calculate_control()
        control = Control()
        control.header = std_msgs.msg.Header()
        (
            control.throttle,
            control.brake,
            control.steer,
            control.shift_gears,
        ) = trajectory_handler.get_control()

        control_pub.publish(control)

        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException as e:
            rospy.logdebug(f"Stopping {rospy.get_name()}, because of interrupt: {e}")
