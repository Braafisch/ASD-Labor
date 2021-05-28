import numpy as np
from rospy import rostime
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
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, t=0.0):
        """Instantiate the object."""
        self.jointStateSub = message_filters.Subscriber("joint_states", JointState)
        self.jointStateCache = message_filters.Cache(self.jointStateSub, 100)
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.t = t

        self.beta = 0
        self.dyaw_dt = 0

    def get_velocity_and_steer(joint_state_2):
        joint_state_dict_v = dict(zip(joint_state_2.name, joint_state_2.velocity))
        joint_state_dict_pos = dict(zip(joint_state_2.name, joint_state_2.position))

        t = joint_state_2.header.stamp.to_sec()
        v = (
            0.25
            * (
                joint_state_dict_v["front_right_wheel_joint"]
                + joint_state_dict_v["rear_left_wheel_joint"]
                + joint_state_dict_v["front_left_wheel_joint"]
                + joint_state_dict_v["rear_right_wheel_joint"]
            )
            * r_wheel
            * 3.6
        )

        steer = joint_state_dict_pos["steering_joint"] / 7.85 * max_steer

        return t, v, steer

    v_get_velocity_and_steer = np.vectorize(get_velocity_and_steer)

    def update_from_joint(self, time_now):
        joint_state = self.jointStateCache.getInterval(
            rostime.Time.from_sec(self.t), time_now
        )
        if joint_state is not None:
            if len(joint_state) > 0:
                t, v, delta = self.v_get_velocity_and_steer(joint_state)
                time_now = time_now.to_sec()
                delta = np.clip(delta, -max_steer, max_steer)

                t = np.diff(np.insert(t, 0, self.t))
                s = v * t

                self.y = self.y + sum(np.sin(delta) * s)
                self.x = self.x + sum(np.cos(delta) * s)
                self.v = v[-1]
                self.yaw = delta[-1]
                self.t = time_now
