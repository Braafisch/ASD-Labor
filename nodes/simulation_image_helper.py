#!/usr/bin/env python3
# coding: utf-8

"""Helper function for re-projecting image points to the road surface and vice
versa.

This module contains a class that can be used in the ASD lab gazebo simulation.
It provides functions for re-projecting image points onto the 3D ground plane
and projecting 3D points on the ground plane on to the image, both using the
known homography between image and ground plane.

Example:

  foo = ClassFoo()
  bar = foo.FunctionBar()
"""

import numpy as np


class SimulationImageHelper:
    """
    This class can be used to project points on the road surface onto the image
    and vice versa. It may be used for detecting lanes in simulation.

    Attributes:
        yHorizon: The y-coordinate of the horizon in the image

    Methods:
        image2road: project 2d image points to 3d road coords
        road2image: project 3d road coords to 2d image points
        distance2imagerow: return which y-coord in the image corresponds to a
        given distance on the 3d road plane
    """

    def __init__(self):
        """
        The class constructor.
        """
        # from calib_image.py
        self.H_I = np.matrix(
            [
                [-7.74914499e-03, 3.95733793e-18, 3.10353257e00],
                [8.56519716e-18, 9.42313768e-05, -1.86052093e00],
                [2.57498016e-18, -2.73825295e-03, 1.00000000e00],
            ]
        )

        self.H_I = self.H_I.reshape(3, 3)
        self.H = self.H_I.I

        X_infinity = np.matrix([0, 10000, 1]).reshape(3, 1)
        self.yHorizon = self.H * X_infinity
        self.yHorizon = int((self.yHorizon[1] / self.yHorizon[2]).item())

    def image2road(self, pts2d):
        """
        Transform a point on the road surface from the image to 3D coordinates
        (in DIN70000, i.e. X to front, Y to left, origin in center of vehicle at
        height 0).
        """
        N, cols = pts2d.shape
        if (cols == 1) and (N == 2):
            # if just two coordinates are given, we make sure we have a (2x1) vector
            pts2d = pts2d.T
        assert (
            cols == 2
        ), "pts2d should be a Nx2 numpy array, but we obtained (%d, %d)" % (N, cols)

        X = self.H_I * np.hstack((pts2d, np.ones((pts2d.shape[0], 1)))).T
        X = X / X[2, :]
        X = X.T

        # the camera is "self.C[2]" in front of the vehicle's center
        return np.hstack((X[:, 1], -X[:, 0]))

    def road2image(self, ptsRd, imSize=None):
        """
        Transform a 3d point on the road surface (in DIN70000, i.e. X to front,
        Y to left) to image coordinates.
        """
        N, cols = ptsRd.shape
        if (cols == 1) and (N == 2):
            # if just two coordinates are given, we make sure we have a (2x1) vector
            ptsRd = ptsRd.T
            N = cols
        assert (
            cols == 2
        ), "ptsRd should be a Nx2 numpy array, but we obtained (%d, %d)" % (N, cols)

        # go back from DIN70000 to our image coordinate system
        pts = np.hstack(
            (
                -ptsRd[:, 1].reshape((-1, 1)),
                ptsRd[:, 0].reshape((-1, 1)),
                np.ones((N, 1)),
            )
        )
        x = self.H * pts.T
        x = x / x[2, :]
        x = x.T

        if imSize is not None:
            valid = np.logical_and((x[:, 0] >= 0), (x[:, 1] >= 0))
            valid = np.logical_and(valid, (x[:, 0] < imSize[1]))
            valid = np.logical_and(valid, (x[:, 1] < imSize[0]))
            valid = np.nonzero(valid)[0].tolist()
            x = x[valid, :]

        return np.hstack((x[:, 0], x[:, 1]))

    def distance2imagerow(self, distance):
        """
        Compute which row in the image corresponds to a distance (in DIN70000).
        """
        p = self.road2image(np.array([[distance, 0]]))
        return p[0, 1]


if __name__ == "__main__":

    imageHelper = SimulationImageHelper()
    print("y-coordinate of horizon in the image: ", imageHelper.yHorizon)

    box_road = np.array([[20, -5], [20, +5], [5, +5], [5, -5]])
    box_image = imageHelper.road2image(box_road)
    box_road_reproject = imageHelper.image2road(box_image)

    print("3d road coords: ", box_road)
    print("2d image coords: ", box_image)
    print("reprojected 3d road coords: ", box_road_reproject)
