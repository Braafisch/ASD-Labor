#!/usr/bin/env python

"""
Helper functions for traffic light visualization and GUI, cf. traffic light simulator.
"""

import rospy
import copy
import numpy as np
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


def getTrafficLightVisualization(x, y, header, color):
    
    markerArray = MarkerArray()
    header.frame_id = '/map' 

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


class TrafficLightInteractionGui:
    """
    This class allows setting the next phase of the traffic light via a context menu.
    Two attributes of this class can be set:
    * automatic: True if phase shall be adjusted based on ellabpsed time
    * event_counter: whenever "Next Phase" is selected from the context menu, this 
        counter is increased by one
    """

    def __init__(self):
        self.server = None
        self.menu_handler = MenuHandler()
        self.event_counter = 0
        self.automatic = True        

    def cbEmpty(self, feedback):
        pass

    def makeMenuMarker(self, x, y):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"        
        int_marker.pose.position = Point(x, y, 2.25)
        int_marker.scale = 1
        int_marker.name = "context_menu"
        int_marker.description = ""

        # make one control using default visuals
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.description = "traffic light\n(right click\nto modify)"
        control.name = "menu_only_control"
        int_marker.controls.append(copy.deepcopy(control))

        # make one control showing a box
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 1.0
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.1

        control.markers.append(marker)
        control.always_visible = True
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.cbEmpty)
        self.menu_handler.apply(self.server, int_marker.name)   

    def cbAutomatic(self, feedback):
        self.automatic = True
        rospy.loginfo('Set traffic light phase transition to automatic.')

    def cbNext(self, feedback):
        self.event_counter += 1
        self.automatic = False        
        rospy.loginfo('Received traffic light state next event trigger (cnt=%d).' % self.event_counter)

    def initInteractionServer(self, x, y, node_name='traffic_light_simulator'):
        self.server = InteractiveMarkerServer(node_name)
        self.menu_handler.insert("Next phase", callback=self.cbNext)
        self.menu_handler.insert("Automatic", callback=self.cbAutomatic)
        self.makeMenuMarker(x, y)
        self.server.applyChanges()
       