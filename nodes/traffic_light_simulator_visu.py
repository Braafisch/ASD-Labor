#!/usr/bin/env python3

"""
Helper functions for traffic light visualization and GUI, cf. traffic light simulator.
"""

import rospy
import copy
import numpy as np
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from car_demo.msg import TrafficLightControl


def getTrafficLightVisualization(x, y, header, color):
    
    markerArray = MarkerArray()
    header.frame_id = 'map' 

    # red
    marker = Marker()
    marker.header = header
    marker.type = marker.SPHERE
    marker.id = 1
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    if color in ['RED', 'RED_YELLOW']:
        marker.color = ColorRGBA(1, 0, 0, 1)
    else:
        marker.color = ColorRGBA(0.3, 0, 0, 1)       
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y 
    marker.pose.position.z = 2.5
    markerArray.markers.append(marker)     
 
    # yellow
    marker = Marker()
    marker.header = header
    marker.type = marker.SPHERE
    marker.id = 2
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    if color in ['YELLOW', 'RED_YELLOW']:
        marker.color = ColorRGBA(1, 1, 0, 1)
    else:
        marker.color = ColorRGBA(0.3, 0.3, 0, 1)       
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y 
    marker.pose.position.z = 2.25 
    markerArray.markers.append(marker)

    # green
    marker = Marker()
    marker.header = header
    marker.type = marker.SPHERE
    marker.id = 3
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    if color in ['GREEN']:
        marker.color = ColorRGBA(0, 1, 0, 1)
    else:
        marker.color = ColorRGBA(0, 0.3, 0, 1) 
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y 
    marker.pose.position.z = 2.0 
    markerArray.markers.append(marker)

    return markerArray


class TrafficLightControlReceiver:
    """
    This class allows receiving the next phase of the traffic light via ros messages.
    Two attributes of this class are available:
    * automatic: True if phase shall be adjusted based on ellabpsed time
    * event_counter: whenever "Next Phase" is selected from the context menu, this 
        counter is increased by one
    """

    def __init__(self):
        self.automatic = False
        self.event_counter = 0

    def initialize(self):
        self.sub = rospy.Subscriber('traffic_light_control', TrafficLightControl,
                                    self.callbackTLControl, queue_size=1)

    def callbackTLControl(self, msg):
        self.automatic = msg.automatic
        self.event_counter = msg.event_counter