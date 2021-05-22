#!/usr/bin/env python3

"""ROS Node for controling vehicle's throttle, brakes and steering.

The task of this node is to move the vehicle along the planned trajectory.
"""

import rospy
import std_msgs.msg
import tf

from car_demo.msg import Control, Trajectory


class TrajectoryHandler:
    """
    A class to subscribe to trajectory updates and print received data.
    """

    # TODO: replace handler behaviour with proper vehicle maneuvering

    def __init__(self):
        # subscribe to lane coefficient updates
        self.trajectory_sub = rospy.Subscriber(
            "trajectory", Trajectory, self._callback, queue_size=1
        )

    def _callback(self, message: Trajectory):
        rospy.loginfo(
            "trajectory: "
            f"x={', '.join(message.x)}, "
            f"y={', '.join(message.y)}, "
            f"theta={', '.join(message.theta)}, "
            f"c={', '.join(message.c)}, "
            f"v={', '.join(message.v)}, "
            f"s={', '.join(message.s)}"
        )


if __name__ == "__main__":
    rospy.init_node("planning_node")
    rate = rospy.Rate(1.0)

    # subscribers
    tf_listener = tf.TransformListener()

    # publishers
    control_pub = rospy.Publisher("prius", Control, queue_size=1)

    # setup lane coefficients handler
    trajectory_handler = TrajectoryHandler()

    # main loop
    time_start = rospy.Time(0)
    while not rospy.is_shutdown():
        time_now = rospy.get_rostime()
        if time_start == rospy.Time(0):
            time_start = time_now

        # TODO: fill message object with proper values
        control = Control()
        control.header = std_msgs.msg.Header()
        control.throttle = 0
        control.brake = 0
        control.steer = 0
        control.shift_gears = Control.NO_COMMAND

        control_pub.publish(control)

        rate.sleep()
