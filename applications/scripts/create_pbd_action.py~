#! /usr/bin/env python

import rospy
import pickle
import actionlib
import fetch_api
import numpy as np
import tf

from geometry_msgs.msg import PoseStamped, Pose
from tf_util import *
from pbd_pose import *
from robot_controllers_msgs.msg import QueryControllerStatesGoal, QueryControllerStatesAction, ControllerState
from ar_track_alvar_msgs.msg import AlvarMarkers

# Variable for if we are running this in simulation. In the simulation, we do not need
# to relax the arm
IN_SIM = False

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

# Gets the current pose of the gripper based on the base_link
def get_current_gripper_pose(listener):
    (trans, rot) = listener.lookupTransform('/base_link', '/wrist_roll_link', rospy.Time(0))
    gr_pose = Pose()
    gr_pose.position.x = trans[0]
    gr_pose.position.y = trans[1]
    gr_pose.position.z = trans[2]
    gr_pose.orientation.x = rot[0]
    gr_pose.orientation.y = rot[1]
    gr_pose.orientation.z = rot[2]
    gr_pose.orientation.w = rot[3]
    return gr_pose


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

    gripper_open = True

    # Init a a tfListener for reading the gripper pose
    listener = tf.TransformListener()
    rospy.sleep(rospy.Duration.from_sec(1))

    # Init the reader for the tags
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)

    # Step 1: Relax the arm
    if not IN_SIM:
        controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
        # Sleep for a second to make sure the client starts up
        rospy.sleep(1.0)
        # The rest of this code is given in the lab
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.STOPPED
        goal.updates.append(state)
        controller_client.send_goal(goal)
        controller_client.wait_for_result()

    # Get the user interface going
    print_user_options()
    running = True
    while running:
        user_input = raw_input("")
        if user_input == "save_pose":
            # Get the current pose with respect to the base_link
            current_gripper_pose = get_current_gripper_pose(listener)
            # Ask the user which frame they would like to save the pose to
            # Eg: base_link, tag 1, tag 2
            print "Please input the frame you would like to save the pose in. The options are:"
            print "     base_link"
            for marker in reader.markers:
                print "     %s" % marker.id

            frame = raw_input("")

            # Check the frame the pose should be saved in
            pose = Pose()
            frame_id = frame
            if frame == "base_link":
                pose = current_gripper_pose
                frame_id = 'base_link'
            else:
                for marker in reader.markers:
                    if frame == str(marker.id):
                        # First, we need to turn the poses into transform matrixes
                        pose_in_base_matrix = pose_to_transform(current_gripper_pose)
                        tag_in_base_matrix = pose_to_transform(marker.pose.pose)
                        # Then take the inverse of the tag in the base fram
                        inv_matrix = np.linalg.inv(tag_in_base_matrix)
                        # Then take the dot product
                        pose_to_save = np.dot( inv_matrix, pose_in_base_matrix )
                        # The target pose is then transformed back into a pose object
                        pose = transform_to_pose(pose_to_save)
                        frame_id = str(marker.id)
            # Create and append the PBD_Pose
            pbd_pose = PBD_Pose(pose, gripper_open, frame)
            sequence.append(pbd_pose)

        if user_input == "open_gripper":
            gripper.open()
            gripper_open = True
        if user_input == "close_gripper":
            gripper.close()
            gripper_open = False
        if user_input == "save_program":
            pickle.dump( sequence, open(save_file_name, "wb") )
        if user_input == "help":
            print_user_options()
        if user_input == "quit":
            running = False


if __name__ == '__main__':
    main()
