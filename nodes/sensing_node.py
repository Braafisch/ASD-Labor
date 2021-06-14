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
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header


class ImageHandler:
    """
    A class to subscribe to an image and create and publish a debug image.
    """

    def __init__(self):
        self.bridge = CvBridge()
        self.image_helper = SimulationImageHelper()
        self.Z_MEst = np.zeros((4, 1))
        self.latest_front_camera_image: Image = None
        #  Select the region which might be interesting for detecting left
        #  and right lane
        self.max_range_m = 15
        self.roi_right_line = np.array(
            [[3, 0], [10, 0], [self.max_range_m, 5], [self.max_range_m, -5], [3, -6]]
        )
        self.roi_left_line = np.array(
            [[3, 0], [10, 0], [self.max_range_m, -5], [self.max_range_m, 5], [3, 6]]
        )
        # subscribers
        self.image_sub = rospy.Subscriber(
            "/prius/front_camera/image_raw", Image, self._callback, queue_size=1
        )
        # The queue_size in the last line is really important! The code here is
        # not thread-safe, so - by all cost - you have to avoid that the
        # callback function is called multiple times in parallel.

        # publishers
        self.pub_dbg_canny = rospy.Publisher("canny_dbg", Image, queue_size=1)
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

    def _callback(self, message: Image):
        self.latest_front_camera_image = message

    def estimate_lane(self, Z_initial):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                self.latest_front_camera_image, "mono8"
            )
        except CvBridgeError as e:
            rospy.logerr("Error in imageCallback: %s", e)

        # Delete horizon/end-of-map
        cv_image = cv2.inRange(cv_image, 240, 255)

        # Detect edges through canny edge detector
        canny_image = cv2.Canny(cv_image, 110, 200)
        self.pub_dbg_canny.publish(self.bridge.cv2_to_imgmsg(canny_image))

        # generate color image and draw box on road
        roi_left_line_im = self.image_helper.road2image(self.roi_left_line)
        roi_right_line_im = self.image_helper.road2image(self.roi_right_line)

        cv_image_color_dbg = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        cv2.polylines(
            cv_image_color_dbg,
            [roi_left_line_im.astype(np.int32)],
            isClosed=True,
            color=(0, 255, 0),
            thickness=8,
        )
        cv2.polylines(
            cv_image_color_dbg,
            [roi_right_line_im.astype(np.int32)],
            isClosed=True,
            color=(255, 0, 0),
            thickness=8,
        )
        # downscale to reduce load
        cv_image_color_dbg = cv2.pyrDown(cv_image_color_dbg)
        try:
            self.pub_dbg_image.publish(self.bridge.cv2_to_imgmsg(cv_image_color_dbg))
        except CvBridgeError as e:
            rospy.logerr(e)

        # Convert detected edges, that are in image coordinates, to road coordinates.
        skip_x = 5
        skip_y = 1
        height_image = canny_image.shape[0]
        crop_bounds_x = np.array([height_image * 0.5, height_image * 0.74]).astype(
            np.int32
        )
        canny_image_cropped_decimated = np.array(
            canny_image[
                crop_bounds_x[0] : crop_bounds_x[1] : skip_x, ::skip_y  # noqa:E203
            ]
        )

        pts_im = np.argwhere(canny_image_cropped_decimated >= 255)
        pts_im[:, 0] *= skip_x
        pts_im[:, 0] += crop_bounds_x[0]
        pts_im[:, 1] *= skip_y
        pts_im = np.roll(pts_im, 1, axis=1)
        distance = np.diff(pts_im[:, 0])
        indi = np.argwhere(((0.5 > distance) | (distance > 7.75))) + 1
        indi = np.reshape(indi, (indi.shape[0],))
        pts_im = pts_im[indi, :]

        pts_road = self.image_helper.image2road(pts_im)

        # Select the region which might be interesting for detecting left and right lane
        lane_left = np.empty((0, 2))
        lane_right = np.empty((0, 2))
        for i in range(pts_road.shape[0]):
            if (
                cv2.pointPolygonTest(
                    self.roi_left_line, (pts_road[i, 0], pts_road[i, 1]), False
                )
                > 0
            ):
                lane_left = np.vstack((lane_left, pts_road[i, :]))
            if (
                cv2.pointPolygonTest(
                    self.roi_right_line, (pts_road[i, 0], pts_road[i, 1]), False
                )
                > 0
            ):
                lane_right = np.vstack((lane_right, pts_road[i, :]))

        self.pub_dbg_pts_lane_left.publish(create_pts_markers(lane_left, g=1))
        self.pub_dbg_pts_lane_right.publish(create_pts_markers(lane_right, b=1))

        self.Z_MEst = Z_initial
        # refine initial estimate via M-Estimator
        if lane_left.size > 2 and lane_right.size > 2:
            same_x_coordinates = np.all(lane_left[:, 0] == lane_left[0, 0]) and np.all(
                lane_right[:, 0] == lane_right[0, 0]
            )
            if not same_x_coordinates:
                self.Z_MEst = self.fit_lane_to_pts(
                    lane_left, lane_right, Z_initial, sigma=0.01, maxIteration=10
                )
                rospy.loginfo(
                    "lane coefficients: "
                    f"W={self.Z_MEst[0][0]}, "
                    f"Y_offset={self.Z_MEst[1][0]}m, "
                    f"dPhi={self.Z_MEst[2][0] * 180.0 / np.pi}deg, "
                    f"c0={self.Z_MEst[3][0]}"
                )
                x_pred, yl_pred, yr_pred = self.compute_lane_from_coeffs(
                    self.Z_MEst, self.max_range_m + 20, step=0.25
                )
                self.pub_dbg_pts_lane_left_pred.publish(
                    create_pred_markers(x_pred, yl_pred, g=1)
                )
                self.pub_dbg_pts_lane_right_pred.publish(
                    create_pred_markers(x_pred, yr_pred, b=1)
                )
        return self.Z_MEst

    def calc_cauchy(self, r, sigma=1):
        """
        Cauchy loss function.

        Args:
            r: residuals
            sigma: expected standard deviation of inliers

        Returns:
            w: vector of weight coefficients
        """
        c = 2.3849 * sigma
        w = 1 / (1 + (r / c) ** 2)
        return w

    def fit_lane_to_pts(self, pL, pR, Z_initial, sigma=1, maxIteration=10):
        """
        M-Estimator for lane coefficients z=(W, Y_offset, Delta_Phi, c0)^T.

        Args:
            pL: [NL, 2]-array of left marking positions (in DIN70000)
            pR: [NR, 2]-array of right marking positions (in DIN70000)
            Z_initial: the initial guess of the parameter vector
            sigma: the expected standard deviation of the inliers
            maxIteration: max number of iterations

        Returns:
            Z: lane coefficients (W, Y_offset, Delta_Phi, c0)
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
            H[pL.shape[0] + i, :] = [-0.5, -1, -u, 1.0 / 2.0 * u2]
            Y[pL.shape[0] + i] = v

        Z = Z_initial
        for i in range(0, maxIteration):
            Z0 = Z
            r = np.dot(H, Z) - Y
            K = np.diag(self.calc_cauchy(r, sigma)[:, 0])
            H_TK = np.linalg.multi_dot([H.T, K, H])
            # Check if H.T*K*H is a singular matrix,
            if np.linalg.det(H_TK) == 0:
                break
            H_inv = np.linalg.inv(H_TK)
            Z = np.linalg.multi_dot([H_inv, H.T, K, Y])
            if np.all(
                [
                    np.isclose(Z[0][0], Z0[0], atol=3e-1),  # Lane Width[m]
                    np.isclose(Z[1][0], Z0[1], atol=4e-2),  # Lateral offset[m]
                    np.isclose(
                        Z[2][0], Z0[2], atol=5e-2
                    ),  # relative angle to lane[rad]
                    np.isclose(Z[3][0], Z0[3], atol=1e-2),  # Parabola coefficient[1]
                ]
            ):
                rospy.logdebug(
                    f"Found M-estimation early, after {i}/{maxIteration} iterations."
                )
                break
        else:
            rospy.logwarn(f"M-estimator hit iteration limit of {maxIteration}.")
        return Z

    def compute_lane_from_coeffs(self, Z, maxDist=60, step=0.5):
        """
        Compute lane points from given parameter vector.

        Args;
            Z: lane coefficients (W, Y_offset, Delta_Phi, c0)
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

    def limit_lane_coeffs(self, Z_MEst):

        # Max of W (lane width) is 5m
        W_min = 3.5
        W_max = 8
        Z_MEst[0][0] = np.clip(Z_MEst[0][0], W_min, W_max)

        # Range of Y_offset is from -2m to 2m
        Yo_min = -2
        Yo_max = 2
        Z_MEst[1][0] = np.clip(Z_MEst[1][0], Yo_min, Yo_max)

        # Range of dPhi from -70 degrees to +70 degrees
        dPhi_min = -60 * np.pi / 180.0
        dPhi_max = 60 * np.pi / 180.0
        Z_MEst[2][0] = np.clip(Z_MEst[2][0], dPhi_min, dPhi_max)

        return Z_MEst


def create_pts_markers(lane, g=0, b=0):
    marker = Marker()
    marker.header = Header()
    marker.header.frame_id = "din70000"
    marker.id = 0
    marker.type = Marker.POINTS
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 1
    for pt in lane:
        p = Point()
        p.x = pt[0, 0]
        p.y = pt[0, 1]
        p.z = 0
        marker.points.append(p)
    return marker


def create_pred_markers(x, y, g=0, b=0):
    marker = Marker()
    marker.header = Header()
    marker.header.frame_id = "din70000"
    marker.id = 0
    marker.type = Marker.POINTS
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 1
    for i in range(len(x)):
        p = Point()
        p.x = x[i]
        p.y = y[i]
        p.z = 0
        marker.points.append(p)
    return marker


if __name__ == "__main__":
    rospy.init_node("sensing_node")
    rate = rospy.Rate(10.0)

    # publishers
    lane_coeff_pub = rospy.Publisher(
        "lane_coefficients", LaneCoefficients, queue_size=1
    )

    # setup image handler
    image_handler = ImageHandler()
    Z_old = np.array([[5], [-0.5], [0.3], [0]])

    seq = 0

    # main loop
    while not rospy.is_shutdown():
        seq += 1

        if image_handler.latest_front_camera_image is not None:
            Z_MEst = image_handler.estimate_lane(Z_initial=Z_old)

            lane_coeff = LaneCoefficients()
            lane_coeff.header.frame_id = "din70000"
            lane_coeff.header.seq = seq
            lane_coeff.header.stamp = rospy.Time.now()
            lane_coeff.W = Z_MEst[0][0]
            lane_coeff.Y_offset = Z_MEst[1][0]
            lane_coeff.dPhi = Z_MEst[2][0]
            lane_coeff.c0 = Z_MEst[3][0]
            lane_coeff_pub.publish(lane_coeff)
            Z_old = image_handler.limit_lane_coeffs(Z_MEst)

        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException as e:
            rospy.logdebug(f"Stopping {rospy.get_name()}, because of interrupt: {e}")
