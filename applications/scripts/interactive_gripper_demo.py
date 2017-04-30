#!/usr/bin/env python

import fetch_api
import rospy
import copy
from fetch_api import *
from visualization_msgs.msg import InteractiveMarker, Marker, InteractiveMarkerControl, InteractiveMarkerFeedback, MenuEntry
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import PoseStamped
from tf_util import *

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self.arm = arm
        self.gripper = gripper
        self.currentPose = None
        self.currentPoseFeasible = False

        self._im_server = im_server
        gripper_im = InteractiveMarker()
        gripper_im.name = "da grip"
        #gripper_im.header.frame_id = "wrist_roll_link"
        gripper_im.header.frame_id = "base_link"
        gripper_im.scale = .5
        gripper_im.pose.orientation.w = 1.0

        markers = init_markers(0.166, 0, 0) # create gripper mesh
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
            # prepare pose and compute ik
            poseStamped = PoseStamped()
            poseStamped.pose = feedback.pose
            poseStamped.header = feedback.header
            success = self.arm.compute_ik(poseStamped)
            
            # change marker color
            gripper_markers = gripper_im.controls[0].markers
            change_marker_colors(gripper_markers, success)
            self.currentPoseFeasible = success
            self.currentPose = poseStamped

            self._im_server.insert(gripper_im)
            self._im_server.applyChanges()
        
        # If the feedback is a menu select, do the coresponding action
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                self.gripper.open()
            elif feedback.menu_entry_id == 2:
                self.gripper.close()
            elif feedback.menu_entry_id == 3:
                kwargs = {
                    'allowed_planning_time': 20,
                    'execution_timeout': 20,
                    'num_planning_attempts': 5,
                    'replan': True,
                }
                if (self.currentPoseFeasible):
                    print "Pose:", self.currentPose
                    self.arm.move_to_pose(self.currentPose, **kwargs)
                else:
                    rospy.logerr("Could not move to pose: current pose infeasible!")


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._im_server = im_server
        self.arm = arm
        self.gripper = gripper

        self.obj_im = InteractiveMarker()
        self.obj_im.name = "da object"
        self.obj_im.header.frame_id = "base_link"
        self.obj_im.scale = 0.5

        self.gripper1_shift = [-0.03, 0, 0]
        self.gripper2_shift = [-0.15, 0, 0]
        self.gripper3_shift = [-0.1, 0, 0.3]

        # prepare parameters for moving the robot later
        self.currentPoseFeasible = False
        self.currentGripperPose1 = None
        self.currentGripperPose2 = None
        self.currentGripperPose3 = None


        marker_control = InteractiveMarkerControl()

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.05
        box_marker.scale.y = 0.05
        box_marker.scale.z = 0.05

        # box_marker.pose.position.x += xOffset
        # box_marker.pose.position.y += yOffset
        # box_marker.pose.position.z += zOffset
        box_marker.color.r = 1
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0
        box_marker.color.a = 0.5

        gripper1 = init_markers(self.gripper1_shift[0], self.gripper1_shift[1], self.gripper1_shift[2]) 
        gripper2 = init_markers(self.gripper2_shift[0], self.gripper2_shift[1], self.gripper2_shift[2]) 
        gripper3 = init_markers(self.gripper3_shift[0], self.gripper3_shift[1], self.gripper3_shift[2])

        marker_control.markers.append(box_marker)
        for marker in gripper1:
            marker_control.markers.append(marker)
        for marker in gripper2:
            marker_control.markers.append(marker)
        for marker in gripper3:
            marker_control.markers.append(marker)
        marker_control.always_visible = True
        marker_control.interaction_mode = InteractiveMarkerControl.MENU

                # Add the menu commands
        entry1 = MenuEntry()
        entry1.id = 1
        entry1.title = 'open'
        entry1.command_type = entry1.FEEDBACK
        self.obj_im.menu_entries.append(entry1)

        entry2 = MenuEntry()
        entry2.id = 2
        entry2.title = 'close'
        entry2.command_type = entry2.FEEDBACK
        self.obj_im.menu_entries.append(entry2)

        entry3 = MenuEntry()
        entry3.id = 3
        entry3.title = 'move_gripper'
        entry3.command_type = entry3.FEEDBACK
        self.obj_im.menu_entries.append(entry3)

        print "Number of markers:", len(marker_control.markers)

        self.obj_im.controls.append(marker_control)

        add_6dof_controls(self.obj_im)

        self._im_server.insert(self.obj_im, feedback_cb = self.handle_feedback)
        self._im_server.applyChanges()

    def start(self):
        pass

    def handle_feedback(self, feedback):
        # If the feedback is a pose update, calculate the IK and update colors accordingly
        obj_im = self._im_server.get("da object")

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # prepare pose and compute ik
            poseStamped = PoseStamped()
            poseStamped.pose = feedback.pose
            poseStamped.header = feedback.header

            # find the poses of the object and the grippers as matrices
            object_in_base_link = pose_to_transform(feedback.pose)
            gripper1_in_object = pose_to_transform(obj_im.controls[0].markers[1].pose)
            gripper1_in_object[0, 3] = gripper1_in_object[0, 3] - 0.166
            gripper2_in_object = pose_to_transform(obj_im.controls[0].markers[4].pose)
            gripper2_in_object[0, 3] = gripper2_in_object[0, 3] - 0.166
            gripper3_in_object = pose_to_transform(obj_im.controls[0].markers[7].pose)
            gripper3_in_object[0, 3] = gripper3_in_object[0, 3] - 0.166

            print "Gripper before any transformations:", obj_im.controls[0].markers[1].pose

            print "Object in base link frame:", feedback.pose
            print "Gripper in object frame:", gripper1_in_object

            # compute the poses of the grippers in the base link frame
            gripper1_in_base_link = transform_to_pose(np.dot(object_in_base_link, gripper1_in_object))
            gripper2_in_base_link = transform_to_pose(np.dot(object_in_base_link, gripper2_in_object))
            gripper3_in_base_link = transform_to_pose(np.dot(object_in_base_link, gripper3_in_object))

            print "Gripper in base link frame as matrix:", np.dot(object_in_base_link, gripper1_in_object)
            print "Gripper in base link frame:", gripper1_in_base_link

            # create the gripper1 stamped pose
            gripper1_pose = PoseStamped()
            gripper1_pose.pose = gripper1_in_base_link
            gripper1_pose.header = copy.deepcopy(feedback.header)

            # create the gripper2 stamped pose
            gripper2_pose = PoseStamped()
            gripper2_pose.pose = gripper2_in_base_link
            gripper2_pose.header = copy.deepcopy(feedback.header)

            # create the gripper3 stamped pose
            gripper3_pose = PoseStamped()
            gripper3_pose.pose = gripper3_in_base_link
            gripper3_pose.header = copy.deepcopy(feedback.header)

            # compute ik's
            success1 = self.arm.compute_ik(gripper1_pose)
            success2 = self.arm.compute_ik(gripper2_pose)
            success3 = self.arm.compute_ik(gripper3_pose)

            # gripper1_pose = copy.deepcopy(poseStamped)
            # gripper1_pose.pose.position.x = gripper1_pose.pose.position.x + self.gripper1_shift[0] - 0.166
            # gripper1_pose.pose.position.y = gripper1_pose.pose.position.y + self.gripper1_shift[1]
            # gripper1_pose.pose.position.z = gripper1_pose.pose.position.z + self.gripper1_shift[2]
            #success1 = self.arm.compute_ik(gripper1_pose)

            # gripper2_pose = copy.deepcopy(poseStamped)
            # gripper2_pose.pose.position.x = gripper2_pose.pose.position.x + self.gripper2_shift[0] - 0.166
            # gripper2_pose.pose.position.y = gripper2_pose.pose.position.y + self.gripper2_shift[1]
            # gripper2_pose.pose.position.z = gripper2_pose.pose.position.z + self.gripper2_shift[2]
            #success2 = self.arm.compute_ik(gripper2_pose)

            # gripper3_pose = copy.deepcopy(poseStamped)
            # gripper3_pose.pose.position.x = gripper3_pose.pose.position.x + self.gripper3_shift[0] - 0.166
            # gripper3_pose.pose.position.y = gripper3_pose.pose.position.y + self.gripper3_shift[1]
            # gripper3_pose.pose.position.z = gripper3_pose.pose.position.z + self.gripper3_shift[2]
            #success3 = self.arm.compute_ik(gripper3_pose)
            
            # change marker colors
            all_markers = obj_im.controls[0].markers
            change_marker_colors(all_markers[1:4], success1)
            change_marker_colors(all_markers[4:7], success2)
            change_marker_colors(all_markers[7:10], success3)

            # update poses
            self.currentPoseFeasible = (success1 and success2 and success3)
            self.currentGripperPose1 = gripper1_pose
            self.currentGripperPose2 = gripper2_pose
            self.currentGripperPose3 = gripper3_pose

            self._im_server.insert(obj_im)
            self._im_server.applyChanges()
        
        # If the feedback is a menu select, do the coresponding action
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                self.gripper.open()
            elif feedback.menu_entry_id == 2:
                self.gripper.close()
            elif feedback.menu_entry_id == 3:
                kwargs = {
                    'allowed_planning_time': 20,
                    'execution_timeout': 20,
                    'num_planning_attempts': 5,
                    'replan': True,
                }
                if (self.currentPoseFeasible):
                    print "Pose:", self.currentGripperPose1
                    self.gripper.open()
                    self.arm.move_to_pose(self.currentGripperPose2, **kwargs)
                    self.arm.move_to_pose(self.currentGripperPose1, **kwargs)
                    self.gripper.close()
                    self.arm.move_to_pose(self.currentGripperPose3, **kwargs)
                else:
                    rospy.logerr("Could not move to pose: current pose infeasible!")

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
def init_markers(xOffset, yOffset, zOffset):
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
    gripper.pose.position.x += xOffset
    gripper.pose.position.y += yOffset
    gripper.pose.position.z += zOffset
    gripper.pose.orientation.w = 1.0
    gripper.color.r = 1
    gripper.color.g = 0.0
    gripper.color.b = 0.0
    gripper.color.a = 0.7

    # # l finger stuff
    l_finger.pose.position.x += xOffset
    l_finger.pose.position.y += -0.05 + yOffset
    l_finger.pose.position.z += zOffset
    l_finger.pose.orientation.w = 1.0

    l_finger.color.r = 1
    l_finger.color.g = 0.0
    l_finger.color.b = 0.0
    l_finger.color.a = 0.7

    # # r finger stuff
    r_finger.pose.position.x += xOffset
    r_finger.pose.position.y += 0.05 + yOffset
    r_finger.pose.position.z += zOffset
    r_finger.pose.orientation.w = 1.0

    r_finger.color.r = 1
    r_finger.color.g = 0.0
    r_finger.color.b = 0.0
    r_finger.color.a = 0.7

    markers = [gripper, l_finger, r_finger]
    return markers


def main():
    rospy.init_node('interactive_gripper_demo')
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=10)
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    teleop = GripperTeleop(arm, gripper, im_server)
    #auto_pick = AutoPickTeleop(arm, gripper, im_server)

    rospy.spin()

if __name__ == "__main__":
    main()