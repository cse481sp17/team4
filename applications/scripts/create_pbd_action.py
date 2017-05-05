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
# For relaxing the arm
from robot_controllers_msgs.msg import QueryControllerStatesGoal, QueryControllerStatesAction, ControllerState
# for the action server
import actionlib
# For the arm and gripper
import fetch_api
# For the AR markers
from ar_track_alvar_msgs.msg import AlvarMarkers
# For the maths
import numpy as np


# Variable for if we are running this in simulation. In the simulation, we do not need
# to relax the arm
IN_SIM = True

class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers


def print_usage():
    print 'Allows the user to move the arm and save a series of poses. The poses are saved to the given file'
    print 'Usage: rosrun applications create_pbd_action.py [FILENAME]'
    print 'Note: The [FILENAME] must be a pick file: (name).p'

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def print_user_options():
    print "Robot arm is ready."
    print "Commands:"
    print "     save_pose : Saves the current arm pose. User will need to specify frame_id"
    print "     open_gripper : Opens the gripper"
    print "     close_gripper : Closes the gripper"
    print "     save_program : Saves the current sequence of poses to the given filename"
    print "     quit : Exits the program. Does not save automatically"
    print "     help : Prints this list of commands"

def main():
    rospy.init_node('create_pbd_action')
    wait_for_time()

    # Check to see the proper args were given
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    save_file_name = argv[1]

    # This is the list of poses that we will save
    sequence = []

    # The Arm and the gripper on the robot
    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()

    # Init a a tfListener for reading the gripper pose
    listener = tf.TransformListener()
    rospy.sleep(rospy.Duration.from_sec(1))

    # Init the reader for the tags
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)

    # Step 1: Relax the arm
    if not IN_SIM:
        # TODO: Get this to work. Right now, it doesn't seem to be receiving result.
        # Maybe double check that the right topic is being published to?

        # Make an Action Client
        controller_client = actionlib.SimpleActionClient('/query_controller_states/cancel', QueryControllerStatesAction)
        # The rest of this code is given in the lab
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.STOPPED
        goal.updates.append(state)
        controller_client.send_goal(goal)
        controller_client.wait_for_result(rospy.Duration.from_sec(5.0))

    # Step 2, Get the user interface going
    print_user_options()
    running = True
    while running:
        user_input = raw_input("")
        if user_input == "save_pose":
            pass
            # TODO: All the steps for save_pose

            # First: Get the current pose with respect to the base_link
            current_gripper_pose = listener.lookupTransform('/base_link', '/wrist_roll_link', rospy.Time(0))

            # Second: As the user which frame they would like to save the pose to
            # Eg: base_link, tag 1, tag 2
            print "Please input the frame you would like to save the pose in. The options are:"
            print "     base_link"
            for marker in reader.markers:
                print "     %s" % marker.id

            frame = raw_input("")
            # Not doing any checks to see if it is an actual fram, we just need to be careful


            # Third: If necessary, do the transform to get the pose in the proper frame
            pose = PoseStamped()
            if frame == "base_link":
                pose.pose = current_gripper_pose
                pose.header.frame_id = 'base_link'
            else:
                for marker in reader.markers:
                    if frame == marker.id:
                        # Now we do the transform math

                        # We want the Pose in the Tag frame. We have Tag in Base frame, and Pose in Base frame
                        # so we need to do the inverse of the Tag in Base frame

                        # First, we need to turn them into transform matrixes. Thankfully we have a method
                        pose_in_base_matrix = tf_util.pose_to_transform(current_gripper_pose)
                        tag_in_base_matrix = tf_util.pose_to_transform(marker.pose.pose)

                        inv_matrix = np.inv(tag_in_base_matrix)

                        pose_to_save = np.dot( inv_matrix, pose_in_base_matrix )

                        pose.pose = pose_to_save
                        pose.header.frame_id = marker.id
            pbd_pose = PBD_Pose(pose, True)

            sequence.append(pbd_pose)
            # Fourth: Create a PBD_Pose and append it to the list "sequence"

        if user_input == "open_gripper":
            gripper.open()
        if user_input == "close_gripper":
            gripper.close()
        if user_input == "save_program":
            pickle.dump( sequence, open(save_file_name, "wb") )
        if user_input == "help":
            print_user_options()
        if user_input == "quit":
            running = False




if __name__ == '__main__':
    main()

# NOTE: After a little chat with Justin it seems there many be another (possibly better)
# way to store the poses. My idea was to store each pose, transformed with respect to its frame.
# His idea seems to be (and this may need double checking) store all the poses in the base_link frame,
# and then store the locations of the tags. Then, we calculate the poses on the fly when we need them in other
# frames. 