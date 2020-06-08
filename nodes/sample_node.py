#!/usr/bin/env python

"""
This is a sample node for the ASD lab.
It will:
* obtain the vehicles speed and position 
* drive the vehicle 
* publish a debug image  
"""

from __future__ import print_function
import rospy
import tf
from car_demo.msg import Control, Trajectory, ControlDebug
import numpy as np
from sensor_msgs.msg import JointState, Image
import message_filters
import std_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError
from simulation_image_helper import SimulationImageHelper

class ImageHandler:
    """
    A class to subscribe to an image and create and publish a debug image.
    """
    def __init__(self):
        self.bridge = CvBridge() 
        self.imageHelper = SimulationImageHelper()  

        # subscribers
        self.imageSub = rospy.Subscriber("/prius/front_camera/image_raw", Image, self.imageCallback, queue_size=1)
        # The queue_size in the last line is really important! The code here is not thread-safe, so
        # - by all cost - you have to avoid that the callback function is called multiple times in parallel.

        # publishers
        self.pubDbgImage = rospy.Publisher("lane_detection_dbg_image", Image, queue_size=1)


    def imageCallback(self, message):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(message, "mono8")
        except CvBridgeError as e:
            rospy.logerr("Error in imageCallback: %s", e)

        # generate color image and draw box on road
        cv_image_color = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

        box_road = np.array([[20, -5], [20, +5], [5, +5], [5, -5]])
        box_image = self.imageHelper.road2image(box_road)
        cv2.polylines(cv_image_color, [box_image.astype(
                np.int32)], isClosed=True, color=(0, 0, 255), thickness=8)

        # downscale to reduce load
        cv_image_color = cv2.pyrDown(cv_image_color)

        try:
            self.pubDbgImage.publish(self.bridge.cv2_to_imgmsg(cv_image_color, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('sample_node')
    rate = rospy.Rate(10.0)

    # subscribers
    listener = tf.TransformListener()
    jointStateSub = message_filters.Subscriber("joint_states", JointState)
    jointStateCache = message_filters.Cache(jointStateSub, 100)
    
    # publishers
    pubControl = rospy.Publisher('prius', Control, queue_size=1)

    # setup image handler
    imageHandler = ImageHandler()
    
    # main loop
    time_start = rospy.Time(0)
    while not rospy.is_shutdown():
        time_now = rospy.get_rostime()
        if time_start == rospy.Time(0):
            time_start = time_now

        # get vehicle position data 
        try:
            (trans, rot) = listener.lookupTransform('/map', '/din70000', rospy.Time(0)) # get latest trafo between world and vehicle
            rpy = tf.transformations.euler_from_quaternion(rot)
            rospy.loginfo("pos: T=%s, rpy=%s" % (str(trans), str(rpy)) )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Could not receive position information!")
            rate.sleep()
            continue
        
        # get vehicle joint data
        joint_state = jointStateCache.getElemBeforeTime(time_now)
        joint_state_dict = dict(zip(joint_state.name, joint_state.velocity))
        v_kmh = 0.25*(joint_state_dict['front_right_wheel_joint'] + \
            joint_state_dict['rear_left_wheel_joint'] + \
            joint_state_dict['front_left_wheel_joint'] + \
            joint_state_dict['rear_right_wheel_joint'])
        rospy.loginfo("v_kmh = " + str(v_kmh))

        # accelerate for 5s, keep speed for 5s, then stop
        dt_sec = time_now.to_sec()-time_start.to_sec()
        if dt_sec < 5:
            throttle_desired = 0.5
        elif dt_sec < 10:
            throttle_desired = 0
        else:
            throttle_desired = -0.5
        rospy.loginfo("throttle = " + str(throttle_desired))

        # send driving commands to vehicle
        command = Control()
        command.header = std_msgs.msg.Header()
        command.header.stamp = time_now
        command.shift_gears = Control.FORWARD
        if throttle_desired > 0:
            command.throttle, command.brake = np.clip(throttle_desired, 0, 1), 0
        else:
            command.throttle, command.brake = 0, np.clip(-throttle_desired, 0, 1)
        command.steer = 0.5
        pubControl.publish(command)

        rate.sleep()
