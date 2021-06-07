import numpy as np
import rospy
import tf
from sensor_msgs.msg import JointState
import message_filters

L = 2.7  # [m] Wheel base of vehicle
veh_dim_x, veh_dim_y = 4.645, 1.76  # [m] size of vehicle (length, width)
max_steer = np.radians(30.0)  # [rad] max steering angle
r_wheel = 0.31265


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class Prius_State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, t=None):
        """Instantiate the object."""
        self.jointStateSub = message_filters.Subscriber("joint_states", JointState)
        self.jointStateCache = message_filters.Cache(self.jointStateSub, 100)
        self.tf_listener = tf.TransformListener()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.t = t

        self.beta = 0
        self.dyaw_dt = 0

    def get_velocity(self):
        joint_state = self.jointStateCache.getElemBeforeTime(self.t)
        if joint_state is not None:
            joint_state_dict_v = dict(zip(joint_state.name, joint_state.velocity))

            v = (
                0.25
                * (
                    joint_state_dict_v["front_right_wheel_joint"]
                    + joint_state_dict_v["rear_left_wheel_joint"]
                    + joint_state_dict_v["front_left_wheel_joint"]
                    + joint_state_dict_v["rear_right_wheel_joint"]
                )
                * r_wheel
            )

            return v
        else:
            return self.v

    def get_postion(self):
        # get vehicle position data
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                "map", "din70000", rospy.Time(0)
            )  # get latest trafo between world and vehicle
            rpy = tf.transformations.euler_from_quaternion(rot)
            rospy.loginfo(
                "pos: T=[%.1f, %.1f, %.1f], rpy=[%.1f, %.1f, %.1f]"
                % (trans[0], trans[1], trans[2], rpy[0], rpy[1], rpy[2])
            )
            return trans[0], trans[1], rpy[2]
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.loginfo("Could not receive position information!")
            return self.x, self.y, self.yaw

    def update(self, time_now):
        self.t = time_now
        self.x, self.y, self.yaw = self.get_postion()
        self.v = self.get_velocity()
