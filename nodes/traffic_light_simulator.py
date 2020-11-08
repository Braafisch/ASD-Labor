#!/usr/bin/env python

"""
This ROS node simulates a traffic light (with random initial state) and publishes a "TrafficLightStatus" message.

To view the state machine, you may use "rosrun smach_viewer smach_viewer.py", but you need to install corresponding 
libraries first using:

sudo apt install ros-melodic-smach ros-melodic-smach-ros ros-melodic-smach-msgs ros-melodic-smach-viewer python-gtk2
"""

import rospy
import smach
from smach_ros import IntrospectionServer
import tf
import random
import numpy as np
from car_demo.msg import TrafficLightStatus



def distPoint2LineSegment(veh_x, veh_y, veh_phi, x1, y1, x2, y2): 
    """
    Find the closest point from vehicle pose (veh_x, veh_y, veh_phi) 
    to a line segment (x1, y1, x2, y2).
    This code is adapted from: 
    https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    """
    px = x2-x1
    py = y2-y1
    norm = px*px + py*py

    u = ((veh_x - x1)*px + (veh_y - y1)*py) / float(norm)
    u = np.clip(u, 0, 1)

    x = x1 + u*px
    y = y1 + u*py

    dx = x - veh_x
    dy = y - veh_y
    dist = np.hypot(dx, dy)
    dphi = np.arctan2(dy, dx) - veh_phi
    dphi = (dphi + np.pi) % (2*np.pi) - np.pi # normalize to -pi..pi

    return (dist, dphi)


# define general traffic light state handler
class TrafficLightState(smach.State):
    def __init__(self, tf_listener, tl_publisher, color='GREEN', duration_s=10, rate_hz=10):
        smach.State.__init__(self, outcomes=['ellapsed', 'aborted'])
        self.rate_hz = rate_hz
        self.duration_s = duration_s
        self.color = color
        self.tf_listener = tf_listener
        self.tl_publisher = tl_publisher
        self.stopline = (35.6, -0.1, 32.8, -2.2)
        self.dist2bumper = 2.75
        
    def execute(self, userdata):
        t_start = rospy.get_time()        
        r = rospy.Rate(self.rate_hz)
        x1, y1, x2, y2 = self.stopline

        rospy.logout('Entering state %s.' % (self.color))

        while rospy.get_time()-t_start < self.duration_s:

            # check if rospy has been terminated 
            if rospy.is_shutdown():
                rospy.logout('Aborting state %s.' % (self.color))
                return 'aborted'

            # get vehicle position data 
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/din70000', rospy.Time(0)) # get latest trafo between world and vehicle
                rpy = tf.transformations.euler_from_quaternion(rot)
                rospy.logdebug("pos: T=%s, rpy=%s" % (str(trans), str(rpy)) )
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Could not receive position information!")
                r.sleep()
                continue

            # compute relative pose to stop line
            dx, dy = self.dist2bumper*np.cos(rpy[2]), self.dist2bumper*np.sin(rpy[2])
            dist, dphi = distPoint2LineSegment(trans[0]+dx, trans[1]+dy, rpy[2], 
                                               x1, y1, x2, y2)
            rospy.logdebug("dist=%.1f, dphi_deg=%.1f" % (dist, dphi*180.0/np.pi))

            # publish current traffic light status
            tl_status = TrafficLightStatus()
            state_dict = { 'GREEN': tl_status.GREEN, 
                           'YELLOW': tl_status.YELLOW, 
                           'RED': tl_status.RED, 
                           'RED_YELLOW': tl_status.RED_YELLOW }
            tl_status.dist_m = dist
            tl_status.heading_rad = dphi
            tl_status.state = state_dict[self.color]
            self.tl_publisher.publish(tl_status)

            r.sleep()

        rospy.logout('Leaving state %s.' % (self.color))

        return 'ellapsed'
        

# main
def main():
    rospy.init_node('traffic_light_simulator')

    # For retrieving vehicle pose
    listener = tf.TransformListener()

    # For publishing the traffic lilght status
    publisher = rospy.Publisher("traffic_light_status", TrafficLightStatus, queue_size=1)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['ellapsed', 'aborted'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GREEN', 
                               TrafficLightState(color='GREEN', duration_s=5, tf_listener=listener, tl_publisher=publisher), 
                               transitions={'ellapsed': 'YELLOW'})
        smach.StateMachine.add('YELLOW', 
                               TrafficLightState(color='YELLOW', duration_s=3, tf_listener=listener, tl_publisher=publisher), 
                               transitions={'ellapsed': 'RED'})
        smach.StateMachine.add('RED', 
                               TrafficLightState(color='RED', duration_s=5, tf_listener=listener, tl_publisher=publisher), 
                               transitions={'ellapsed': 'RED_YELLOW'})
        smach.StateMachine.add('RED_YELLOW', 
                               TrafficLightState(color='RED_YELLOW', duration_s=3, tf_listener=listener, tl_publisher=publisher), 
                               transitions={'ellapsed': 'GREEN'})

    # Set random initial state
    state_list = ['GREEN', 'YELLOW', 'RED', 'RED_YELLOW']
    sm.set_initial_state([random.choice(state_list)])

    # For debugging and visualisation
    sis = IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()