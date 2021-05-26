#!/usr/bin/env python3

"""ROS Node for controling vehicle's throttle, brakes and steering.

The task of this node is to move the vehicle along the planned trajectory.
"""

from numpy.lib.function_base import _calculate_shapes
import rospy
import std_msgs.msg
from sensor_msgs.msg import JointState
import message_filters
import tf
import numpy as np

from car_demo.msg import Control, Trajectory
#from vehicle_model import State, normalize_angle, veh_dim_x, veh_dim_y, L

class TrajectoryHandler:
    """
    A class to subscribe to trajectory updates and print received data.
    """
    # TODO: replace handler behaviour with proper vehicle maneuvering
    def __init__(self):
        self.k = 0.7
        self.throttle = 0
        self.brake = 0
        self.steer = 0
        self.jointStateSub = message_filters.Subscriber("joint_states", JointState)
        self.jointStateCache = message_filters.Cache(self.jointStateSub, 100)
        # subscribe to lane coefficient updates
        self.trajectory_sub = rospy.Subscriber(
            "trajectory", Trajectory, self._callback, queue_size=1
        )

    def get_velocity(self):
        # get vehicle joint data
        joint_state = self.jointStateCache.getElemBeforeTime(rospy.get_rostime())
        joint_state_dict = dict(zip(joint_state.name, joint_state.velocity))
        r_wheel = 0.31265
        v = (
            0.25
            * (
                joint_state_dict["front_right_wheel_joint"]
                + joint_state_dict["rear_left_wheel_joint"]
                + joint_state_dict["front_left_wheel_joint"]
                + joint_state_dict["rear_right_wheel_joint"]
            )
            * r_wheel
        )
        rospy.loginfo("v = " + str(v))
        return v

    def calculate_control(self, v, trajec):
        ai = trajec.v[0] - v

        dx_vec = 1.35 - np.asarray(trajec.x).reshape([-1,1])
        assert dx_vec.shape == (len(dx_vec), 1)
        dy_vec = -np.asarray(trajec.y).reshape([-1,1])
        assert dy_vec.shape == (len(dy_vec), 1)
        dist = np.hstack([dx_vec, dy_vec])
        front_axle_vec = [np.cos(trajec.theta[0] + np.pi / 2),
                      np.sin(trajec.theta[0] + np.pi / 2)]
        error_front_axle = np.dot(dist[0, :], front_axle_vec)

        de = trajec.theta[0] + np.arctan2(-self.k * error_front_axle, v)
        return ai, de

    def _callback(self, message: Trajectory):
        
        rospy.loginfo(
            "trajectory: "
            f"x={str(message.x)}, "
            f"y={str(message.y)}, "
            f"theta={str(message.theta)}, "
            f"c={str(message.c)}, "
            f"v={str(message.v)}, "
            f"s={str(message.s)}"
        )
        
        a, delta = self.calculate_control(self.get_velocity(), message)
        if a >= 0:
            self.throttle = 1.0
            self.brake = 0
        else:
            self.brake = 1.0
            self.throttle = 0
        self.steer = 3 * delta / np.pi
        rospy.loginfo("steer = " + str(self.steer))

    def get_control(self):
        return self.throttle, self.brake, self.steer
        

if __name__ == "__main__":
    rospy.init_node("control_node")
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
        control.throttle, control.brake, control.steer = trajectory_handler.get_control() 
        control.shift_gears = Control.FORWARD

        control_pub.publish(control)

        rate.sleep()
