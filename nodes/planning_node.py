#!/usr/bin/env python3

"""ROS Node for planning a trajectory based on lane coefficients produced by
sensory nodes.

The task of this node is to plan a trajectory (vector of x,y-coordinates with
angles and velocity associated) the vehicle should follow.
"""

import numpy as np
import rospy
import std_msgs.msg
import tf

from car_demo.msg import LaneCoefficients, Trajectory, TrafficLightStatus


class LaneCoefficientsHandler:
    """
    A class to subscribe to lane coefficient updates and print received data.
    """

    # TODO: replace handler behaviour with proper trajectory planning

    def __init__(self):
        # subscribe to lane coefficient updates
        self.lane_coeff_sub = rospy.Subscriber(
            "lane_coefficients", LaneCoefficients, self._callback, queue_size=1
        )

    def _callback(self, message: LaneCoefficients):
        rospy.loginfo(
            "lane coefficients: "
            f"W={message.W:.3f}, "
            f"Y_offset={message.Y_offset:.3f}, "
            f"dPhi={message.dPhi:.3f}, "
            f"c0={message.c0:.3f}"
        )


class TrafficLightHandler:
    """
    A class to subscribe to traffic light status and print received data.
    """

    # TODO: replace handler behaviour with proper trajectory planning

    def __init__(self):
        # subscribe to traffic light status
        self.traffic_light_sub = rospy.Subscriber(
            "traffic_light_status", TrafficLightStatus, self._callback, queue_size=1
        )

    def _callback(self, message):
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


if __name__ == "__main__":
    rospy.init_node("planning_node")
    rate = rospy.Rate(1.0)

    # subscribers
    tf_listener = tf.TransformListener()

    # publishers
    trajectory_pub = rospy.Publisher("trajectory", Trajectory, queue_size=1)

    # setup lane coefficients handler
    lane_coeff_handler = LaneCoefficientsHandler()

    # setup traffic light handler
    traffic_light_handler = TrafficLightHandler()

    # main loop
    time_start = rospy.Time(0)
    while not rospy.is_shutdown():
        time_now = rospy.get_rostime()
        if time_start == rospy.Time(0):
            time_start = time_now

        # TODO: fill message object with proper values
        trajectory = Trajectory()
        trajectory.header = std_msgs.msg.Header()
        trajectory.x = 0
        trajectory.y = 0
        trajectory.theta = 0
        trajectory.c = 0
        trajectory.v = 0
        trajectory.s = 0

        trajectory_pub.publish(trajectory)

        rate.sleep()
