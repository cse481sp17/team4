#! /usr/bin/env python

import rospy
# For the poses
from geometry_msgs.msg import PoseStamped
# For calculating the transforms
from tf_util import *
# For saving the actions
import pickle
# For storing the poses
from pbd_pose import *
# For the arm and gripper
from fetch_api import Arm, Gripper

import actionlib

def print_usage():
    print 'Allows the user to move the arm and save a series of poses. The poses are saved to the given file'
    print 'Usage: rosrun applications execute_pbd_action.py [FILENAME]'
    print 'Note: The [FILENAME] must be a pick file: (name).p'

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node("execute_pbd_action")
    wait_for_time()

    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    load_file_name = argv[1]

    # Step 0: Start the arm controller (similar to stopping it on create_pbd_action)
    # controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    # goal = QueryControllerStatesGoal()
    # state = ControllerState()
    # state.name = 'arm_controller/follow_joint_trajectory'
    # state.state = ControllerState.RUNNING
    # goal.updates.append(state)
    # controller_client.send_goal(goal)
    # controller_client.wait_for_result()

    # Step 1: Load the sequence of poses
    sequence = pickle.load( open(load_file_name, "rb") )
    # The Arm and the gripper on the robot
    gripper = Gripper()
    arm = Arm()

    # Step 3: For each pose in the list, move to that pose
    
    for pose in sequence:
        # TODO: The transforming and moving to each pose
        print pose.pose_stamped


if __name__ == '__main__':
    main()