#!/usr/bin/env python3

"""
This ROS node simulates a traffic light (with random initial state) and
publishes a "TrafficLightStatus" message. You can also add the topic
"traffic_light_visu" in rviz to display the traffic light.

To view the state machine, you may use "rosrun smach_viewer smach_viewer.py",
but you need to install corresponding libraries first using:

sudo apt install \
    ros-noetic-smach \
    ros-melodic-smach-ros \
    ros-noetic-smach-msgs \
    ros-noetic-smach-viewer \
    python-gtk2
"""

import rospy
import smach
from smach_ros import IntrospectionServer
import tf
import numpy as np
from visualization_msgs.msg import MarkerArray

from car_demo.msg import TrafficLightStatus
from traffic_light_simulator_visu import (
    getTrafficLightVisualization,
    TrafficLightControlReceiver,
)


# global instance of traffic light interaction
tl_control = TrafficLightControlReceiver()

# stop line coordinates: (xRight, yRight, xLeft, yLeft)
STOP_LINE = (35.6, -0.1, 29.7, -4.6)

# max distance for valid stop lines
MAX_STOP_LINE_DIST = 25


def distPoint2LineSegment(veh_x, veh_y, veh_phi, xr, yr, xl, yl):
    """
    Find the closest point from vehicle pose (veh_x, veh_y, veh_phi)
    to a line segment (xr, yr, xl, yl).
    This code is adapted from:
    https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    """
    px = xl - xr
    py = yl - yr
    norm = px * px + py * py

    u = ((veh_x - xr) * px + (veh_y - yr) * py) / float(norm)
    u = np.clip(u, 0, 1)

    x = xr + u * px
    y = yr + u * py

    dx = x - veh_x
    dy = y - veh_y
    dist = np.hypot(dx, dy)

    # normal in driving direction
    nx, ny = py, -px
    if dx * nx + dy * ny < 0:
        dist = -dist  # we passed the stop line already

    dphi = veh_phi - np.arctan2(ny, nx)
    dphi = (dphi + np.pi) % (2 * np.pi) - np.pi  # normalize to -pi..pi

    return (dist, dphi)


# define general traffic light state handler
class TrafficLightState(smach.State):
    def __init__(
        self,
        tf_listener,
        tl_publisher,
        tlvis_publisher,
        color="GREEN",
        duration_s=10,
        rate_hz=10,
    ):
        smach.State.__init__(self, outcomes=["ellapsed", "aborted"])
        self.rate_hz = rate_hz
        self.duration_s = duration_s
        self.color = color
        self.tf_listener = tf_listener
        self.tl_publisher = tl_publisher
        self.tlvis_publisher = tlvis_publisher
        # NOTE: the order is important here (xRight, yRight, xLeft, yLeft)
        self.stopline = STOP_LINE
        self.dist2bumper = 2.75

    def execute(self, userdata):
        t_start = rospy.get_time()
        r = rospy.Rate(self.rate_hz)
        x1, y1, x2, y2 = self.stopline
        curr_cnt = tl_control.event_counter

        rospy.logout("Entering state %s." % (self.color))

        while (
            (tl_control.automatic) and (rospy.get_time() - t_start < self.duration_s)
        ) or ((not tl_control.automatic) and (tl_control.event_counter == curr_cnt)):

            # check if rospy has been terminated
            if rospy.is_shutdown():
                rospy.logout("Aborting state %s." % (self.color))
                return "aborted"

            # get vehicle position data
            try:
                (trans, rot) = self.tf_listener.lookupTransform(
                    "map", "din70000", rospy.Time(0)
                )  # get latest trafo between world and vehicle
                rpy = tf.transformations.euler_from_quaternion(rot)
                rospy.logdebug("pos: T=%s, rpy=%s" % (str(trans), str(rpy)))
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                rospy.logwarn("Could not receive position information!")
                r.sleep()
                continue

            # compute relative pose to stop line
            dx, dy = self.dist2bumper * np.cos(rpy[2]), self.dist2bumper * np.sin(
                rpy[2]
            )
            dist, dphi = distPoint2LineSegment(
                trans[0] + dx, trans[1] + dy, rpy[2], x1, y1, x2, y2
            )
            rospy.loginfo("dist=%.1f, dphi_deg=%.1f" % (dist, dphi * 180.0 / np.pi))

            # publish current traffic light status
            tl_status = TrafficLightStatus()
            state_dict = {
                "GREEN": tl_status.GREEN,
                "YELLOW": tl_status.YELLOW,
                "RED": tl_status.RED,
                "RED_YELLOW": tl_status.RED_YELLOW,
            }
            tl_status.header.stamp = rospy.get_rostime()
            tl_status.dist_m = dist
            tl_status.dphi_rad = dphi

            if (
                np.abs(tl_status.dist_m) > MAX_STOP_LINE_DIST
            ):  # traffic light too far away
                tl_status.dist_m = tl_status.INVALID
                tl_status.dphi_rad = tl_status.INVALID

            tl_status.state = state_dict[self.color]
            self.tl_publisher.publish(tl_status)

            # publish traffic light visualization
            tl_visualization = getTrafficLightVisualization(
                x1, y1, tl_status.header, self.color
            )
            self.tlvis_publisher.publish(tl_visualization)

            try:
                r.sleep()
            except rospy.exceptions.ROSInterruptException as e:
                rospy.logdebug(
                    f"Stopping {rospy.get_name()}, because of interrupt: {e}"
                )

        rospy.logout("Leaving state %s." % (self.color))

        return "ellapsed"


# main
def main():
    rospy.init_node("traffic_light_simulator")

    # For retrieving vehicle pose
    listener = tf.TransformListener()

    # For retrieving traffic light control commands
    tl_control.initialize()

    # For publishing the traffic lilght status and visualization
    status_publisher = rospy.Publisher(
        "traffic_light_status", TrafficLightStatus, queue_size=1
    )
    visu_publisher = rospy.Publisher("traffic_light_visu", MarkerArray, queue_size=1)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["ellapsed", "aborted"])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(
            "GREEN",
            TrafficLightState(
                color="GREEN",
                duration_s=5,
                tf_listener=listener,
                tl_publisher=status_publisher,
                tlvis_publisher=visu_publisher,
            ),
            transitions={"ellapsed": "YELLOW"},
        )
        smach.StateMachine.add(
            "YELLOW",
            TrafficLightState(
                color="YELLOW",
                duration_s=3,
                tf_listener=listener,
                tl_publisher=status_publisher,
                tlvis_publisher=visu_publisher,
            ),
            transitions={"ellapsed": "RED"},
        )
        smach.StateMachine.add(
            "RED",
            TrafficLightState(
                color="RED",
                duration_s=5,
                tf_listener=listener,
                tl_publisher=status_publisher,
                tlvis_publisher=visu_publisher,
            ),
            transitions={"ellapsed": "RED_YELLOW"},
        )
        smach.StateMachine.add(
            "RED_YELLOW",
            TrafficLightState(
                color="RED_YELLOW",
                duration_s=3,
                tf_listener=listener,
                tl_publisher=status_publisher,
                tlvis_publisher=visu_publisher,
            ),
            transitions={"ellapsed": "GREEN"},
        )

    # Set random initial state
    # state_list = ['GREEN', 'YELLOW', 'RED', 'RED_YELLOW']
    # sm.set_initial_state([random.choice(state_list)])
    sm.set_initial_state(["RED"])

    # For debugging and visualisation of state machine
    sis = IntrospectionServer("server_name", sm, "/SM_ROOT")
    sis.start()

    # Execute SMACH plan
    sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
