#! /usr/bin/env python

import rospy
import pickle
import numpy as np
import actionlib

from pbd_pose import *
from fetch_api import Arm, Gripper
from geometry_msgs.msg import PoseStamped
from tf_util import *
from ar_track_alvar_msgs.msg import AlvarMarkers
from robot_controllers_msgs.msg import QueryControllerStatesGoal, QueryControllerStatesAction, ControllerState


def print_usage():
    print 'Allows the user to move the arm and save a series of poses. The poses are saved to the given file'
    print 'Usage: rosrun applications execute_pbd_action.py [FILENAME]'
    print 'Note: The [FILENAME] must be a pick file: (name).p'

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

def main():
    rospy.init_node("execute_pbd_action")
    wait_for_time()

    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    load_file_name = argv[1]

    # TODO: Get this to work. I'm not sure why it isn't. Probably the same problem as the create file

    controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    # Sleep for a second to make sure the client starts up
    rospy.sleep(1.0)
    goal = QueryControllerStatesGoal()
    state = ControllerState()
    state.name = 'arm_controller/follow_joint_trajectory'
    state.state = ControllerState.RUNNING
    goal.updates.append(state)
    controller_client.send_goal(goal)
    controller_client.wait_for_result()

    # Load the sequence of poses
    sequence = pickle.load( open(load_file_name, "rb") )
    # The Arm and the gripper on the robot
    gripper = Gripper()
    arm = Arm()

    gripper_open = True

    # Init the reader for the tags
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)

    # Sleep to make sure the markers show up
    rospy.sleep(1.0)

    # For each pose in the list, move to that pose
    # and open the gripper as necessary
    for pbd_pose in sequence:
        move_pose = PoseStamped()
        move_pose.header.frame_id = 'base_link'
        if pbd_pose.frame == 'base_link':
            move_pose.pose = pbd_pose.pose
        else:
            for marker in reader.markers:
                if pbd_pose.frame == str(marker.id):
                    # Transform the pose to be in the base_link frame
                    pose_in_tag_frame = pose_to_transform(pbd_pose.pose)
                    tag_in_base_frame = pose_to_transform(marker.pose.pose)

                    target_matrix = np.dot(tag_in_base_frame, pose_in_tag_frame)

                    target_pose = transform_to_pose(target_matrix)

                    move_pose.pose = target_pose

        err = arm.move_to_pose(move_pose) # TODO: maybe not fail? or just use better sequences of poses less likely to fail
        if err is not None:
            print "Error in move to pose: ", err
        # Check the gripper to open/close
        if pbd_pose.gripper_open != gripper_open:
            if gripper_open == True:
                gripper.close()
                gripper_open = False
            else:
                gripper.open()
                gripper_open = True




if __name__ == '__main__':
    main()