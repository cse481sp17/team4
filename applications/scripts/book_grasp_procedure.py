#! /usr/bin/env python

import rospy
import fetch_api
import pickle

from pbd_pose import *
from geometry_msgs.msg import PoseStamped, Pose
from tf_util import *
from ar_track_alvar_msgs.msg import AlvarMarkers

from perception_new.srv import *

import math

import copy

from moveit_python import PlanningSceneInterface

TARGET_ID = 13
#INSERT_GRASP_POSES = "/home/team4/catkin_ws/src/cse481c/applications/scripts/testBookInsertPull2.p"
INSERT_GRASP_POSES = "/home/team4/catkin_ws/src/cse481c/library_bot/nodes/real_robot_holder_grab3.p"

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

    # Init the planning scene for collisions
    planning_scene = PlanningSceneInterface('base_link')

    print "waiting for service...."
    rospy.wait_for_service('get_spines')
    print "found service!"

    get_spine_poses = rospy.ServiceProxy('get_spines', GetSpineLocations)
    response = get_spine_poses()
    planning_scene.addBox('surface', (response.surface_x_size - 0.17), (response.surface_y_size), response.surface_z_size,
       response.surface_pose.position.x, response.surface_pose.position.y, response.surface_pose.position.z)

    gripper.open()
    gripper_open = True

    target_marker_pose = None
    check = 0
    while target_marker_pose == None and check < 100
        # If the fiducial was not seen, try again
        rospy.sleep(0.1)
        check += 1
        print "Searching for fiducial...."
        for marker in reader.markers:
            if TARGET_ID == marker.id:
                target_marker_pose = marker.pose.pose

    print "Surface position z"
    print response.surface_pose.position.z
    print "target marker position z"
    print target_marker_pose.position.z

    # This is the same as the pbd action stuff, not making any changes at the moment
    for pbd_pose in sequence:
        move_pose = PoseStamped()
        move_pose.header.frame_id = 'base_link'
        if pbd_pose.frame == 'base_link':
            move_pose.pose = pbd_pose.pose
        else:
            # for marker in reader.markers:
            #     if TARGET_ID == marker.id:
            print "Calculating pose relative to marker...."

            # Transform the pose to be in the base_link frame
            pose_in_tag_frame = pose_to_transform(pbd_pose.pose)
            #tag_in_base_frame = pose_to_transform(marker.pose.pose)
            tag_in_base_frame = pose_to_transform(target_marker_pose)

            target_matrix = np.dot(tag_in_base_frame, pose_in_tag_frame)

            target_pose = transform_to_pose(target_matrix)

            move_pose.pose = target_pose

        rospy.sleep(1)
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

        # add the surface for collisons
      #  planning_scene.addBox('surface', response.surface_x_size, response.surface_y_size, response.surface_z_size,
      #   response.surface_pose.position.x, response.surface_pose.position.y, response.surface_pose.position.z)

        # debugging line
        for pose in spine_poses:
            print pose
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # After this we calculate the spine_pose closest to the fiducial. I will test that if I can get the service call working
    target_fiducial = None
    check = 0
    while target_marker_pose == None and check < 100
        # If the fiducial was not seen, try again
        rospy.sleep(0.1)
        check += 1
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
    grasp_pose.pose = copy.deepcopy(closest_pose)
    # Offset because the arm is moved relative to the wrist roll Joint
    grasp_pose.pose.position.x -= (0.166 - 0.035)
    grasp_pose.pose.orientation.w = 1

    pre_grasp = PoseStamped()
    pre_grasp.header.frame_id = 'base_link'
    pre_grasp.pose = copy.deepcopy(closest_pose)
    pre_grasp.pose.position.x = closest_pose.position.x - (0.166 + 0.065)
    pre_grasp.pose.position.y = closest_pose.position.y
    pre_grasp.pose.position.z = closest_pose.position.z

    post_grasp = PoseStamped()
    post_grasp.header.frame_id = 'base_link'
    post_grasp.pose = copy.deepcopy(closest_pose)
    #post_grasp.pose.position.x = closest_pose.position.x - (0.166 + 0.05)
    post_grasp.pose.position.x -= (0.166 - 0.04)
    post_grasp.pose.position.y = closest_pose.position.y
    post_grasp.pose.position.z = closest_pose.position.z + 0.10

    post_grasp2 = PoseStamped()
    post_grasp2.header.frame_id = 'base_link'
    post_grasp2.pose = copy.deepcopy(closest_pose)
    post_grasp2.pose.position.x = closest_pose.position.x - (0.166 + 0.185) # 0.17
    post_grasp2.pose.position.y = closest_pose.position.y
    post_grasp2.pose.position.z = closest_pose.position.z + 0.10

    carry_position = PoseStamped()
    carry_position.header.frame_id = 'base_link'
    carry_position.pose.position.x = 0.012627533637
    #carry_position.pose.position.y = -0.540503621101
    carry_position.pose.position.y = 0.540503621101 - 0.05
    carry_position.pose.position.z = 0.967533946037
    carry_position.pose.orientation.x = -0.736985862255
    carry_position.pose.orientation.y = 0.0
    carry_position.pose.orientation.z = 0.0
    carry_position.pose.orientation.w = 0.675908148289

    # Oookay, If I want this position I need joint state reader

    curled_pose = PoseStamped()
    curled_pose.header.frame_id = 'base_link'
    curled_pose.pose.position.x = 0.0120114460588
    curled_pose.pose.position.y = 0.169908821583
    curled_pose.pose.position.z = 0.5732229352
    curled_pose.pose.orientation.x = -0.484937250614
    curled_pose.pose.orientation.y = -0.513127207756
    curled_pose.pose.orientation.z = -0.468707561493
    curled_pose.pose.orientation.w = 0.53089505434

    # pre_grasp.pose.position.x -= (0.166 + 0.05)
    # pre_grasp.pose.orientation.w = 1

    # Note: This is only the position of the spine, not any sort of pre or post grasp
    err = arm.move_to_pose(pre_grasp)
    print "Error in move to pregrasp pose: ", err

    rospy.sleep(1.0)

    err = arm.move_to_pose(grasp_pose)
    print "Error in move to grasp pose: ", err

    gripper.close()
    gripper_open = False

    err = arm.move_to_pose(post_grasp)
    print "Error in move to postgrasp pose: ", err

    err = arm.move_to_pose(post_grasp2)
    print "Error in move to postgrasp2 pose: ", err

    err = arm.move_to_pose(carry_position)
    print "Error in move to carry_position pose: ", err


    # At the end remove collision objects
    planning_scene.removeCollisionObject('surface')

    # err = arm.move_to_pose(curled_pose)
    # print "Error in move to curled_pose pose: ", err




if __name__ == '__main__':
    main()
