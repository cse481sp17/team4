#!/usr/bin/env python

from Driver import Driver

import fetch_api

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

import rospy

class DestinationMarker(object):
    def __init__(self, server, x, y, name, driver):
        # ... Initialization, marker creation, etc. ...
        self._x = x
        self._y = y
        self._server = server
        self._name = name
        self._driver = driver

        int_marker = InteractiveMarker()
        int_marker.name = self._name
        int_marker.header.frame_id = "odom"
        int_marker.pose.position.x = self._x
        int_marker.pose.position.y = self._y

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.pose.orientation.w = 1
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.45
        box_marker.scale.z = 0.45
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        button_control.markers.append(box_marker)
        int_marker.controls.append( button_control )

        self._server.insert(int_marker, self._callback)
        self._server.applyChanges()

        #driver.start()


    def _callback(self, feedback_msg):
        # TODO: How do you get the interactive marker given msg.marker_name?
        # See the InteractiveMarkerServer documentation
        interactive_marker = self._server.get(feedback_msg.marker_name)
        position = interactive_marker.pose.position
        if feedback_msg.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo('User clicked {} at {}, {}, {}'.format(feedback_msg.marker_name, position.x, position.y, position.z))
            self._driver.goal = position # Updates the Driver's goal.
        #driver.start()

if __name__ == "__main__":
    rospy.init_node('fetch_marker_test')
    base = fetch_api.Base()
    server = InteractiveMarkerServer("simple_marker")
    driver = Driver(base)
    marker1 = DestinationMarker(server, 2, 2, 'dest1', driver)
    marker2 = DestinationMarker(server, 1, 0, 'dest2', driver)
    marker3 = DestinationMarker(server, 3, -1, 'dest3', driver)
    print "Before start"
    driver.start()
    print "after start"