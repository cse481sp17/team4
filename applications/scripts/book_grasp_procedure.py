#! /usr/bin/env python

import rospy
import fetch_api
import pickle

from pbd_pose import *
from geometry_msgs.msg import PoseStamped, Pose
from tf_util import *
from ar_track_alvar_msgs.msg import AlvarMarkers

from perception.srv import *

import math

TARGET_ID = 13
INSERT_GRASP_POSES = "testBookInsertPull2.p"

class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def calculate_euclidean_distance(pose1, pose2):
    x_sub = (pose1.position.x - pose2.position.x) ** 2
    y_sub = (pose1.position.y - pose2.position.y) ** 2
    z_sub = (pose1.position.z - pose2.position.z) ** 2
    result = math.sqrt(x_sub + y_sub + z_sub)
    return result

def main():
    rospy.init_node("book_grasp_procedure")
    wait_for_time()

    # First, load the poses for the fiducial insert movement
    sequence = pickle.load( open(INSERT_GRASP_POSES, "rb") )
    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()
    gripper_open = True

    # # Init the reader for the tags
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)
    rospy.sleep(0.5)

    # This is the same as the pbd action stuff, not making any changes at the moment
    for pbd_pose in sequence:
        move_pose = PoseStamped()
        move_pose.header.frame_id = 'base_link'
        if pbd_pose.frame == 'base_link':
            move_pose.pose = pbd_pose.pose
        else:
            for marker in reader.markers:
                if TARGET_ID == marker.id:
                    print "Calculating pose relative to marker...."

                    # Transform the pose to be in the base_link frame
                    pose_in_tag_frame = pose_to_transform(pbd_pose.pose)
                    tag_in_base_frame = pose_to_transform(marker.pose.pose)

                    target_matrix = np.dot(tag_in_base_frame, pose_in_tag_frame)

                    target_pose = transform_to_pose(target_matrix)

                    move_pose.pose = target_pose

        err = arm.move_to_pose(move_pose)
        print "Error in move to pose: ", err
        # Check the gripper to open/close
        if pbd_pose.gripper_open != gripper_open:
            if gripper_open == True:
                gripper.close()
                gripper_open = False
            else:
                gripper.open()
                gripper_open = True

    print "Take this opportunity to change to a different mock point cloud, if necessary"
    user_input = raw_input("Press enter to continue")

    # Now, we make a request to the perception side of things
    spine_poses = []

    print "waiting for service...."
    rospy.wait_for_service('get_spines')
    print "found service!"
    try:
        get_spine_poses = rospy.ServiceProxy('get_spines', GetSpineLocations)
        response = get_spine_poses()
        spine_poses = response.spine_poses
        # debugging line
        for pose in spine_poses:
            print pose
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # After this we calculate the spine_pose closest to the fiducial. I will test that if I can get the service call working
    target_fiducial = None
    for marker in reader.markers:
        if marker.id == TARGET_ID:
            target_fiducial = marker

    print target_fiducial.id
    closest_pose = None
    min_dist = float('inf')
    for pose in spine_poses:
        distance = calculate_euclidean_distance(pose, target_fiducial.pose.pose)
        if distance < min_dist:
            min_dist = distance
            closest_pose = pose

    print "Pose closest to target fiducial"
    print closest_pose

    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = 'base_link'
    grasp_pose.pose = closest_pose
    # Offset because the arm is moved relative to the wrist roll Joint
    grasp_pose.pose.position.x -= 0.166
    grasp_pose.pose.orientation.w = 1

    # Note: This is only the position of the spine, not any sort of pre or post grasp
    err = arm.move_to_pose(grasp_pose)
    print "Error in move to pose: ", err







if __name__ == '__main__':
    main()