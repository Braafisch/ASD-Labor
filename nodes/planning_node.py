#!/usr/bin/env python3

"""ROS Node for planning a trajectory based on lane coefficients produced by
sensory nodes.

The task of this node is to plan a trajectory (vector of x,y-coordinates with
angles and velocity associated) the vehicle should follow.
"""

import numpy as np
import rospy
import tf

from car_demo.msg import LaneCoefficients, Trajectory, TrafficLightStatus
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import MarkerArray, Marker

MAX_DIST = 50
STEP_SIZE = 0.5


class LaneCoefficientsHandler:
    """
    A class to subscribe to lane coefficient updates and print received data.
    """

    def __init__(self):
        # subscribe to lane coefficient updates
        self._lane_coeff_sub = rospy.Subscriber(
            "lane_coefficients", LaneCoefficients, self._callback, queue_size=1
        )
        self.latest_lane_coeff: LaneCoefficients = None

    def _callback(self, message: LaneCoefficients):
        rospy.loginfo(
            "lane coefficients: "
            f"W={message.W:.3f}, "
            f"Y_offset={message.Y_offset:.3f}, "
            f"dPhi={message.dPhi:.3f}, "
            f"c0={message.c0:.3f}"
        )
        self.latest_lane_coeff = message


class TrafficLightHandler:
    """
    A class to subscribe to traffic light status and print received data.
    """

    def __init__(self):
        # subscribe to traffic light status
        self._traffic_light_sub = rospy.Subscriber(
            "traffic_light_status", TrafficLightStatus, self._callback, queue_size=1
        )
        self.latest_traffic_light_status: TrafficLightStatus = None

    def _callback(self, message: TrafficLightStatus):
        state_dict = {
            message.GREEN: "GREEN",
            message.YELLOW: "YELLOW",
            message.RED: "RED",
            message.RED_YELLOW: "RED_YELLOW",
        }
        rospy.loginfo(
            "traffic light: "
            f"state={state_dict[message.state]}, "
            f"dist={message.dist_m:.1f}m, "
            f"dphi={message.dphi_rad * 180.0 / np.pi:.1f}deg"
        )
        self.latest_traffic_light_status = message


def create_trajectory(
    lane_coeff: LaneCoefficients, max_dist: float = 50, step: float = 0.5
) -> Trajectory:
    x = np.arange(0, max_dist, step)
    y = np.empty_like(x)
    Z = np.array([lane_coeff.W, lane_coeff.Y_offset, lane_coeff.dPhi, lane_coeff.c0])

    for i, u in enumerate(x):
        u2 = u ** 2
        y[i] = np.dot(np.array([0, -1, -u, 0.5 * u2]), Z)

    delta = np.diff(np.column_stack((x, y)), axis=0)
    theta = np.pad(np.arctan2(delta[:, 1], delta[:, 0]), pad_width=(0, 1), mode="edge")
    c = np.pad(np.diff(theta) / step, pad_width=(0, 1), mode="edge")
    velocity = np.full_like(x, 3)  # static velocity
    s = np.linalg.norm(delta)

    trajectory = Trajectory()
    trajectory.header = lane_coeff.header
    trajectory.header.frame_id = "base_link"
    trajectory.x = x
    trajectory.y = y
    trajectory.theta = theta
    trajectory.c = c
    trajectory.v = velocity
    trajectory.s = s
    return trajectory


def create_debug_marker(
    id: int, x: float, y: float, yaw: float, header: Header, arrow_size: float = 0.5
) -> Marker:
    marker = Marker()
    marker.header = header
    marker.id = id
    marker.action = Marker.ADD
    marker.type = Marker.ARROW
    marker.pose.position = Point(x=x, y=y)
    q = quaternion_from_euler(0, 0, yaw)
    marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    marker.color = ColorRGBA(r=1, g=0, b=0, a=1)
    marker.scale = Vector3(x=arrow_size, y=arrow_size * 0.1, z=arrow_size * 0.1)
    return marker


if __name__ == "__main__":
    rospy.init_node("planning_node")
    rate = rospy.Rate(1.0)

    # subscribers
    tf_listener = tf.TransformListener()

    # publishers
    trajectory_pub = rospy.Publisher("trajectory", Trajectory, queue_size=1)
    trajectory_dbg_pub = rospy.Publisher("trajectory_dbg", MarkerArray, queue_size=1)

    # setup lane coefficients handler
    lane_coeff_handler = LaneCoefficientsHandler()

    # setup traffic light handler
    traffic_light_handler = TrafficLightHandler()

    # main loop
    while not rospy.is_shutdown():
        # Naive trajectory planning, based souley on interpolating the
        # LaneCoefficients until a traffic light is found.

        lane_coeff = lane_coeff_handler.latest_lane_coeff

        if lane_coeff is not None:
            trajectory = create_trajectory(
                lane_coeff, max_dist=MAX_DIST, step=STEP_SIZE
            )
            trajectory_pub.publish(trajectory)

            marker_array = MarkerArray()
            marker_array.markers = [
                create_debug_marker(
                    i, x, y, th, trajectory.header, arrow_size=STEP_SIZE
                )
                for i, (x, y, th) in enumerate(
                    zip(trajectory.x, trajectory.y, trajectory.theta)
                )
            ]

            trajectory_dbg_pub.publish(marker_array)

        rate.sleep()
