#!/usr/bin/env python3

"""ROS Node for sensing the environment using the vehicles front camera.

The task of this node is to detect the left and right lane markings on the
street, using the front facing camera.
"""

import cv2
import numpy as np
import rospy
import std_msgs.msg

from .simulation_image_helper import SimulationImageHelper
from car_demo.msg import LaneCoefficients
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ImageHandler:
    """
    A class to subscribe to an image and create and publish a debug image.
    """

    # TODO: replace handler behaviour with proper trajectory planning

    def __init__(self):
        self.bridge = CvBridge()
        self.image_helper = SimulationImageHelper()

        # subscribers
        self.image_sub = rospy.Subscriber(
            "/prius/front_camera/image_raw", Image, self._callback, queue_size=1
        )
        # The queue_size in the last line is really important! The code here is
        # not thread-safe, so - by all cost - you have to avoid that the
        # callback function is called multiple times in parallel.

        # publishers
        self.pub_dbg_image = rospy.Publisher(
            "lane_detection_dbg_image", Image, queue_size=1
        )

    def _callback(self, message):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(message, "mono8")
        except CvBridgeError as e:
            rospy.logerr("Error in imageCallback: %s", e)

        # generate color image and draw box on road
        cv_image_color = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

        box_road = np.array([[20, -5], [20, +5], [5, +5], [5, -5]])
        box_image = self.image_helper.road2image(box_road)
        cv2.polylines(
            cv_image_color,
            [box_image.astype(np.int32)],
            isClosed=True,
            color=(0, 0, 255),
            thickness=8,
        )

        # downscale to reduce load
        cv_image_color = cv2.pyrDown(cv_image_color)

        try:
            self.pub_dbg_image.publish(
                self.bridge.cv2_to_imgmsg(cv_image_color, "bgr8")
            )
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == "__main__":
    rospy.init_node("sensing_node")
    rate = rospy.Rate(1.0)

    # publishers
    lane_coeff_pub = rospy.Publisher(
        "lane_coefficients", LaneCoefficients, queue_size=1
    )

    # setup image handler
    image_handler = ImageHandler()

    # main loop
    time_start = rospy.Time(0)
    while not rospy.is_shutdown():
        time_now = rospy.get_rostime()
        if time_start == rospy.Time(0):
            time_start = time_now

        # TODO: fill message object with proper values
        lane_coeff = LaneCoefficients()
        lane_coeff.header = std_msgs.msg.Header()
        lane_coeff.W = 0
        lane_coeff.Y_offset = 0
        lane_coeff.dPhi = 0
        lane_coeff.c0 = 0

        lane_coeff_pub.publish(lane_coeff)

        rate.sleep()
