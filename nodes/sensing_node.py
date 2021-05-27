#!/usr/bin/env python3

"""ROS Node for sensing the environment using the vehicles front camera.

The task of this node is to detect the left and right lane markings on the
street, using the front facing camera.
"""

import cv2
import numpy as np
import rospy

from simulation_image_helper import SimulationImageHelper
from car_demo.msg import LaneCoefficients
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header


class ImageHandler:
    """
    A class to subscribe to an image and create and publish a debug image.
    """

    # TODO: replace handler behavior with proper trajectory planning
    def __init__(self):
        self.bridge = CvBridge()
        self.image_helper = SimulationImageHelper()
        self.Z_MEst = np.zeros((4, 1))
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
        self.pub_dbg_pts_lane_left = rospy.Publisher(
            "pts_lane_left_dbg", Marker, queue_size=1
        )
        self.pub_dbg_pts_lane_right = rospy.Publisher(
            "pts_lane_right_dbg", Marker, queue_size=1
        )
        self.pub_dbg_pts_lane_left_pred = rospy.Publisher(
            "pts_lane_left_pred_dbg", Marker, queue_size=1
        )
        self.pub_dbg_pts_lane_right_pred = rospy.Publisher(
            "pts_lane_right_pred_dbg", Marker, queue_size=1
        )

    def _callback(self, message):
        print("Enter callback function of Image Handler")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(message, "mono8")
        except CvBridgeError as e:
            rospy.logerr("Error in imageCallback: %s", e)

        # detect lines
        cv_image_color = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        cv2.imwrite("cv_im_color.jpg", cv_image_color)
        #  Canny-Edge Detector
        canny_image = cv2.Canny(cv_image_color, 110, 200)
        cv2.imwrite("canny_image.jpg", canny_image)
        #  Convert detected edges, that are in image coordinates, to road coordinates.
        pts_im = np.array([])
        for x, y in np.ndindex(canny_image.shape):
            if canny_image[x, y] == 255:
                pts_im = np.append(pts_im, [y, x])
        pts_im = pts_im.reshape((int(len(pts_im) / 2), 2))
        pts_road = self.image_helper.image2road(pts_im)

        #  Select the region which might be interesting for the lane detection.
        max_range_m = 40
        roi_right_line = np.array(
            [[3, 0], [15, 0], [max_range_m, 3], [max_range_m, -5], [3, -5]]
        )
        roi_left_line = np.array(
            [[3, 0], [15, 0], [max_range_m, -3], [max_range_m, 5], [3, 5]]
        )
        lane_left = np.empty((0, 2))
        lane_right = np.empty((0, 2))

        for i in range(pts_road.shape[0]):
            if (
                cv2.pointPolygonTest(
                    roi_left_line, (pts_road[i, 0], pts_road[i, 1]), False
                )
                > 0
            ):
                lane_left = np.vstack((lane_left, pts_road[i, :]))
            if (
                cv2.pointPolygonTest(
                    roi_right_line, (pts_road[i, 0], pts_road[i, 1]), False
                )
                > 0
            ):
                lane_right = np.vstack((lane_right, pts_road[i, :]))

        self.pub_dbg_pts_lane_left.publish(setMarker(lane_left, g=1))
        self.pub_dbg_pts_lane_right.publish(setMarker(lane_right, b=1))

        # generate color image and draw box on road
        cv_image_color = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        roi_left_line_im = self.image_helper.road2image(roi_left_line)
        roi_right_line_im = self.image_helper.road2image(roi_right_line)
        cv2.polylines(
            cv_image_color,
            [roi_left_line_im.astype(np.int32), roi_right_line_im.astype(np.int32)],
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

        # initial estimate: straight road
        Z_initial = np.array([5, -0.5, 0.3, 0]).T

        # refine initial estimate via M-Estimator
        self.Z_MEst = self.MEstimator_lane_fit(
            lane_left, lane_right, Z_initial, sigma=0.2, maxIteration=10
        )
        x_pred, yl_pred, yr_pred = LS_lane_compute(
            self.Z_MEst, max_range_m + 20, step=0.25
        )
        self.pub_dbg_pts_lane_left_pred.publish(setMarkerPred(x_pred, yl_pred, g=1))
        self.pub_dbg_pts_lane_right_pred.publish(setMarkerPred(x_pred, yr_pred, b=1))
        print("Exit callback function of Image Handler")

    def get_Z_MEst(self):
        return self.Z_MEst

    def Cauchy(self, r, sigma=1):
        """
        Cauchy loss function.

        Args:
            r: resiudals
            sigma: expected standard deviation of inliers

        Returns:
            w: vector of weight coefficients
        """
        c = 2.3849 * sigma
        w = 1 / (1 + (r / c) ** 2)
        return w

    def MEstimator_lane_fit(self, pL, pR, Z_initial, sigma=1, maxIteration=10):
        """
        M-Estimator for lane coeffients z=(W, Y_offset, Delta_Phi, c0)^T.

        Args:
            pL: [NL, 2]-array of left marking positions (in DIN70000)
            pR: [NR, 2]-array of right marking positions (in DIN70000)
            Z_initial: the initial guess of the parameter vector
            sigma: the expected standard deviation of the inliers
            maxIteration: max number of iterations

        Returns:
            Z: lane coeffients (W, Y_offset, Delta_Phi, c0)
        """

        H = np.zeros((pL.shape[0] + pR.shape[0], 4))  # design matrix
        Y = np.zeros((pL.shape[0] + pR.shape[0], 1))  # noisy observations

        # fill H and Y for left line points
        for i in range(pL.shape[0]):
            u, v = pL[i, 0], pL[i, 1]
            u2 = u * u
            H[i, :] = [0.5, -1, -u, 1.0 / 2.0 * u2]
            Y[i] = v

        # fill H and Y for right line points
        for i in range(pR.shape[0]):
            u, v = pR[i, 0], pR[i, 1]
            u2 = u * u
            u3 = u2 * u
            H[pL.shape[0] + i, :] = [-0.5, -1, -u, 1.0 / 2.0 * u2]
            Y[pL.shape[0] + i] = v

        Z = Z_initial
        for _ in range(0, maxIteration):
            r = np.dot(H, Z) - Y
            w = self.Cauchy(r, sigma)
            K = np.diag(w[:, 0])
            H_inv = np.linalg.inv(np.linalg.multi_dot([H.T, K, H]))
            Z = np.linalg.multi_dot([H_inv, H.T, K, Y])

        return Z


def LS_lane_compute(Z, maxDist=60, step=0.5):
    """
    Compute lane points from given parameter vector.

    Args;
        Z: lane coeffients (W, Y_offset, Delta_Phi, c0)
        maxDist[=60]: distance up to which lane shall be computed
        step[=0.5]: step size in x-direction (in m)

    Returns:
        (x_pred, yl_pred, yr_pred): x- and y-positions of left and
            right lane points
    """
    x_pred = np.arange(0, maxDist, step)
    yl_pred = np.zeros_like(x_pred)
    yr_pred = np.zeros_like(x_pred)

    for i in range(x_pred.shape[0]):
        u = x_pred[i]
        u2 = u * u
        yl_pred[i] = np.dot(np.array([0.5, -1, -u, 1.0 / 2.0 * u2]), Z)
        yr_pred[i] = np.dot(np.array([-0.5, -1, -u, 1.0 / 2.0 * u2]), Z)

    return (x_pred, yl_pred, yr_pred)


def setMarker(lane, g=0, b=0):
    ptsMarker = Marker()
    ptsMarker.header = Header()
    ptsMarker.header.frame_id = "base_link"
    ptsMarker.id = 0
    ptsMarker.type = Marker.POINTS
    ptsMarker.scale.x = 0.1
    ptsMarker.scale.y = 0.1
    ptsMarker.color.g = g
    ptsMarker.color.b = b
    ptsMarker.color.a = 1
    for pt in lane:
        p = Point()
        p.x = pt[0, 0]
        p.y = pt[0, 1]
        p.z = 0
        ptsMarker.points.append(p)
    return ptsMarker


def setMarkerPred(x, y, g=0, b=0):
    ptsMarker = Marker()
    ptsMarker.header = Header()
    ptsMarker.header.frame_id = "base_link"
    ptsMarker.id = 0
    ptsMarker.type = Marker.POINTS
    ptsMarker.scale.x = 0.1
    ptsMarker.scale.y = 0.1
    ptsMarker.color.g = g
    ptsMarker.color.b = b
    ptsMarker.color.a = 1
    for i in range(len(x)):
        p = Point()
        p.x = x[i]
        p.y = y[i]
        p.z = 0
        ptsMarker.points.append(p)
    return ptsMarker


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

        lane_coeff = LaneCoefficients()
        lane_coeff.header = Header()
        Z_MEst = image_handler.get_Z_MEst()
        lane_coeff.W = Z_MEst[0][0]
        lane_coeff.Y_offset = Z_MEst[1][0]
        lane_coeff.dPhi = Z_MEst[2][0]
        lane_coeff.c0 = Z_MEst[3][0]

        lane_coeff_pub.publish(lane_coeff)

        rate.sleep()
