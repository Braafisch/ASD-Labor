#!/usr/bin/env python3

"""ROS Node for sensing the environment using the vehicles front camera.

The task of this node is to detect the left and right lane markings on the
street, using the front facing camera.
"""

import cv2
import numpy as np
import rospy
import std_msgs.msg
from matplotlib import pyplot as plt

from simulation_image_helper import SimulationImageHelper
from car_demo.msg import LaneCoefficients
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ImageHandler:
    """
    A class to subscribe to an image and create and publish a debug image.
    """

    # TODO: replace handler behavior with proper trajectory planning

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

        # detect lines
        cv_image_color = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        cv2.imwrite('cv_im_color.jpg',cv_image_color)
        #  Canny-Edge Detector
        canny_image = cv2.Canny(cv_image_color,110,200)
        cv2.imwrite('canny_image.jpg',canny_image)
        #  Convert detected edges, that are in image coordinates, to road coordinates.
        pts_im = np.array([])
        for x, y in np.ndindex(canny_image.shape):
            if canny_image[x,y] == 255:
                pts_im = np.append(pts_im,[y,x])
        pts_im = pts_im.reshape((int(len(pts_im)/2),2))
        pts_road = self.image_helper.image2road(pts_im)

        #  Select the region which is be interesting for the lane detection.
        max_range_m = 20
        h_im = int(0.74*cv_image_color.shape[0])
        h_road = self.image_helper.image2road(np.array([[0, h_im]]))[0,1]
        print(h_im,h_road)
        roi_right_line= np.array([
            [3, 0], 
            [9, 0],
            [max_range_m, 7],
            [max_range_m, -10],
            [3, -10] ])
        roi_left_line = np.array([
            [3, 0],
            [9, 0],
            [max_range_m, -7],
            [max_range_m, 10],
            [3, 10] ])
        lane_left = np.empty((0,2))
        lane_right = np.empty((0,2))

        for i in range(pts_road.shape[0]):
            if cv2.pointPolygonTest(roi_left_line, (pts_road[i,0], pts_road[i,1]), False) > 0:
                lane_left = np.vstack((lane_left, pts_road[i,:]))
            if cv2.pointPolygonTest(roi_right_line, (pts_road[i,0], pts_road[i,1]), False) > 0:
                lane_right = np.vstack((lane_right, pts_road[i,:]))

        # generate color image and draw box on road
        cv_image_color = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        box_road = np.array([[20, -5], [20, +5], [5, +5], [5, -5]])
        box_image = self.image_helper.road2image(box_road)
        #box_image = np.array([region_of_interest_vertices], np.int32)
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
