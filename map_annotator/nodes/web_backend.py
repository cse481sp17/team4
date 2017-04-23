#!/usr/bin/env python

import fetch_api
import rospy
from web_nav_manager import WebNavigationManager 
from map_annotator.msg import PoseNames, UserAction
from geometry_msgs.msg import Pose

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

import move_base_msgs.msg

nav_manager = None
server = None
nav_manager_pub = None

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class PoseMarker:

    def __init__(self, server, name, pose=None):
        # ... Initialization, marker creation, etc. ...
        self.pose = pose
        if pose == None:
            self.pose = Pose()
        
        self._server = server
        self._name = name

        int_marker = InteractiveMarker()
        int_marker.name = self._name
        int_marker.header.frame_id = "map"
        int_marker.pose = self.pose

        box_marker = Marker()
        box_marker.type = Marker.ARROW
        box_marker.pose.orientation.w = 1
        box_marker.scale.x = 0.5
        box_marker.scale.y = 0.1
        box_marker.scale.z = 0.1
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        name_marker = Marker()
        name_marker.type = Marker.TEXT_VIEW_FACING
        name_marker.scale.x = 0.2
        name_marker.scale.y = 0.2
        name_marker.scale.z = 0.2
        name_marker.color.r = 0.0
        name_marker.color.g = 1.0
        name_marker.color.b = 0.0
        name_marker.color.a = 0.8
        name_marker.text = self._name

        move_control = InteractiveMarkerControl()

        move_control.orientation.w = 1
        move_control.orientation.x = 0
        move_control.orientation.y = 1
        move_control.orientation.z = 0

        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_control.always_visible = True
        move_control.markers.append(box_marker)

        move_control.markers.append(name_marker)

        int_marker.controls.append( move_control )

        rotation_control = InteractiveMarkerControl()
        rotation_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        move_control.always_visible = True

        rotation_control.orientation.w = 1
        rotation_control.orientation.x = 0
        rotation_control.orientation.y = 1
        rotation_control.orientation.z = 0



        int_marker.controls.append( rotation_control )

        self._server.insert(int_marker)
        self._server.setCallback(int_marker.name, self.callback, InteractiveMarkerFeedback.POSE_UPDATE)
        self._server.applyChanges()

    
    def callback(self, feedback_msg):
        # TODO: How do you get the interactive marker given msg.marker_name?
        # See the InteractiveMarkerServer documentation

        interactive_marker = self._server.get(feedback_msg.marker_name)
        position = feedback_msg.pose

        if feedback_msg.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo('User clicked {}, new pose x/y/z: {}, {}, {}'.format(feedback_msg.marker_name, position.position.x, position.position.y, position.position.z))
            self.pose = position # Updates the pose to the new position
            nav_manager.savePose(feedback_msg.marker_name, position)
            self._server.applyChanges() # update list of posenames


def process_command(data):
    if data.command == data.CREATE:
        # create nav marker, callback for 
        print "Create called"
        marker = PoseMarker(server, data.name)
        nav_manager.savePose(data.name, marker.pose)
        # Update list of pose names here cuz hard
        msg = PoseNames()
        msg.names = nav_manager.listPoses()
        nav_manager_pub.publish(msg)
    elif data.command == data.DELETE:
        # delete marker
        server.erase(data.name)
        nav_manager.deletePose(data.name)
        msg = PoseNames()
        msg.names = nav_manager.listPoses()
        nav_manager_pub.publish(msg)
        server.applyChanges() # update list of posenames
    elif data.command == data.GOTO:
        # goto marker
        nav_manager.goto(data.name)
    else:
        print "nothing done, command: ", data.command
        pass


def main():
    rospy.init_node('map_annotator_navigation')
    wait_for_time()
    global nav_manager
    global nav_manager_pub
    nav_manager = WebNavigationManager()
    nav_manager.loadPoses()

    # Sleeping so publisher has time to get started
    rospy.sleep(rospy.Duration.from_sec(2))

    global server
    server = InteractiveMarkerServer("simple_marker")

    for poseName in nav_manager.poses:
        marker = PoseMarker(server, poseName, nav_manager.poses[poseName])

    nav_manager_pub = rospy.Publisher('pose_names', PoseNames, latch=True, queue_size=10)
    user_action_sub = rospy.Subscriber('/user_actions', UserAction, process_command)

    # Publish the pre-existing markers
    msg = PoseNames()
    msg.names = nav_manager.listPoses()
    nav_manager_pub.publish(msg)

    # Publish markers as they're updated
    # while not rospy.is_shutdown():
     #   rospy.sleep(.05)
     #   msg = PoseNames()
     #   msg.names = nav_manager.listPoses()
     #   nav_manager_pub.publish(msg)


    rospy.spin()


if __name__ == '__main__':
    main()
