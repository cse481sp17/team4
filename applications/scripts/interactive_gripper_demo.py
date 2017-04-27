#!/usr/bin/env python

import fetch_api
import rospy
import copy
from fetch_api import *
from visualization_msgs.msg import InteractiveMarker, Marker, InteractiveMarkerControl, InteractiveMarkerFeedback, MenuEntry
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import PoseStamped

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self.arm = arm
        self.gripper = gripper

        self._im_server = im_server
        gripper_im = InteractiveMarker()
        gripper_im.name = "da grip"
        gripper_im.header.frame_id = "wrist_roll_link"
        gripper_im.scale = .5
        gripper_im.pose.orientation.w = 1.0

        markers = init_markers() # create gripper mesh
        # Adds the gripper markers to the interactive markers
        marker_control = InteractiveMarkerControl()
        for marker in markers:
            marker_control.markers.append(marker)
        marker_control.always_visible = True
        marker_control.interaction_mode = InteractiveMarkerControl.MENU
        
        gripper_im.controls.append(marker_control)

        # Add the 6 degree of freedom controls
        add_6dof_controls(gripper_im)

        # Add the menu commands
        entry1 = MenuEntry()
        entry1.id = 1
        entry1.title = 'open'
        entry1.command_type = entry1.FEEDBACK
        gripper_im.menu_entries.append(entry1)

        entry2 = MenuEntry()
        entry2.id = 2
        entry2.title = 'close'
        entry2.command_type = entry2.FEEDBACK
        gripper_im.menu_entries.append(entry2)

        entry3 = MenuEntry()
        entry3.id = 3
        entry3.title = 'move_gripper'
        entry3.command_type = entry3.FEEDBACK
        gripper_im.menu_entries.append(entry3)

        self._im_server.insert(gripper_im, feedback_cb = self.handle_feedback)
        self._im_server.applyChanges()


    def start(self):
        pass

    def handle_feedback(self, feedback):
        gripper_im = copy.deepcopy(self._im_server.get('da grip'))
        # If the feedback is a pose update, calculate the IK and update colors accordingly
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            poseStamped = PoseStamped()
            poseStamped.pose = feedback.pose
            poseStamped.header = feedback.header
            success = self.arm.compute_ik(poseStamped)
            if success:
                gripper_markers = gripper_im.controls[0].markers
                change_marker_colors(gripper_markers, True)
            else:
                gripper_markers = gripper_im.controls[0].markers
                change_marker_colors(gripper_markers, False)
            # Here is where the issue is happening, with the marker disappearing
            self._im_server.insert(gripper_im)
            self._im_server.applyChanges()
        # If the feedback is a menu select, do the coresponding action
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                self.gripper.open()
            elif feedback.menu_entry_id == 2:
                self.gripper.close()


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._im_server = im_server
        self.obj_im = InteractiveMarker()
        self._im_server.insert(self.obj_im, feedback_cb = self.handle_feedback)

    def start(self):
        pass

    def handle_feedback(self, feedback):
        pass

# changes the colors of the gripper according to whether the pose is legal
def change_marker_colors(markers, green=True):
    if green:
        for marker in markers:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
    else: #red
        for marker in markers:
            marker.color.r = 1
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

# Gives 6 degree of freedom controls to the given interactive marker
def add_6dof_controls(interactive_marker):
    control = InteractiveMarkerControl()

    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = 'rotate_x'
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    interactive_marker.controls.append(control)

    control = copy.deepcopy(control)
    control.name = 'move_x'
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    interactive_marker.controls.append(control)

    control = copy.deepcopy(control)
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = 'rotate_z'
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    interactive_marker.controls.append(control)

    control = copy.deepcopy(control)
    control.name = 'move_z'
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    interactive_marker.controls.append(control)

    control = copy.deepcopy(control)
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = 'rotate_y'
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    interactive_marker.controls.append(control)

    control = copy.deepcopy(control)
    control.name = 'move_y'
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    interactive_marker.controls.append(control)

# Creates the 3 mesh markers that make up the gripper
def init_markers(pose_stamped=None):
    """
    Returns either 1 InteractiveMarker or list of 3 Markers
    tbd
    """
    gripper = Marker()
    l_finger = Marker()
    r_finger = Marker()

    # setup meshes
    gripper.type = Marker.MESH_RESOURCE
    gripper.mesh_resource = GRIPPER_MESH
    l_finger.type = Marker.MESH_RESOURCE
    l_finger.mesh_resource = L_FINGER_MESH
    r_finger.type = Marker.MESH_RESOURCE
    r_finger.mesh_resource = R_FINGER_MESH

    # # gripper
    gripper.pose.position.x += .166
    gripper.color.r = 1
    gripper.color.g = 0.0
    gripper.color.b = 0.0
    gripper.color.a = 1.0

    # # l finger stuff
    l_finger.pose.position.x += .166
    l_finger.pose.position.y -= .05

    l_finger.color.r = 1
    l_finger.color.g = 0.0
    l_finger.color.b = 0.0
    l_finger.color.a = 1.0

    # # r finger stuff
    r_finger.pose.position.x += .166
    r_finger.pose.position.y += .05

    r_finger.color.r = 1
    r_finger.color.g = 0.0
    r_finger.color.b = 0.0
    r_finger.color.a = 1.0

    markers = [gripper, l_finger, r_finger]
    return markers


def main():
    rospy.init_node('interactive_gripper_demo')
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=10)
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, im_server)
    teleop.start()
    auto_pick.start()
    rospy.spin()

if __name__ == "__main__":
    main()