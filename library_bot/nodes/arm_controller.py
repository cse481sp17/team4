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

INSERT_GRASP_POSES = "/home/team4/catkin_ws/src/cse481c/library_bot/nodes/real_robot_holder_grab3.p"

# Calls arm to pull out target tray and take book

class ArmController(object):

    def __init__(self):
        # set arm and gripper, and fiducial id
        self.arm = fetch_api.Arm()
        self.gripper = fetch_api.Gripper()
        # Will need to change this to something real at some point
        #self.target_id = None# target_id

        self.planning_scene = PlanningSceneInterface('base_link')

        # First, load the poses for the fiducial insert movement
        self.sequence = pickle.load( open(INSERT_GRASP_POSES, "rb") )
        self.gripper_open = True

        # # Init the reader for the tags
        self.reader = ArTagReader()
        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.reader.callback)

    def open_gripper(self):
        self.gripper.open()
        self.gripper_open = True

    def add_bounding_box(self):
        # Init the planning scene for collisions
        

        print "waiting for service...."
        rospy.wait_for_service('get_spines')
        print "found service!"

        get_spine_poses = rospy.ServiceProxy('get_spines', GetSpineLocations)
        response = get_spine_poses()
        planning_scene.addBox('surface', (response.surface_x_size - 0.17), response.surface_y_size, response.surface_z_size,
           response.surface_pose.position.x, response.surface_pose.position.y, response.surface_pose.position.z)
    

    def grab_tray(self, target_id):
        # This is the same as the pbd action stuff, not making any changes at the moment
        self.gripper.open()
        self.gripper_open = True
        target_marker_pose = None
        check = 0
        print "Searching for fiducial...."
        while target_marker_pose == None and check < 100:
            # If the fiducial was not seen, try again
            rospy.sleep(0.1)
            check += 1
            for marker in self.reader.markers:
                if target_id == marker.id:
                    target_marker_pose = marker.pose.pose

        if target_marker_pose == None:
            print "Fiducial not found"


        everError = None

        # This is the same as the pbd action stuff, not making any changes at the moment
        for pbd_pose in self.sequence:
            move_pose = PoseStamped()
            move_pose.header.frame_id = 'base_link'
            if pbd_pose.frame == 'base_link':
                move_pose.pose = pbd_pose.pose
            else:
                # for marker in reader.markers:
                #     if target_id == marker.id:
                print "Calculating pose relative to marker...."

                # Transform the pose to be in the base_link frame
                pose_in_tag_frame = pose_to_transform(pbd_pose.pose)
                #tag_in_base_frame = pose_to_transform(marker.pose.pose)
                tag_in_base_frame = pose_to_transform(target_marker_pose)

                target_matrix = np.dot(tag_in_base_frame, pose_in_tag_frame)

                target_pose = transform_to_pose(target_matrix)

                move_pose.pose = target_pose

            rospy.sleep(1)
            err = self.arm.move_to_pose(move_pose)
            print "Error in move to pose: ", err
            if err != None:
                return False
            # Check the gripper to open/close
            if pbd_pose.gripper_open != self.gripper_open:
                if self.gripper_open == True:
                    self.gripper.close()
                    self.gripper_open = False
                else:
                    self.gripper.open()
                    self.gripper_open = True
        return True

    def find_grasp_pose(self, target_id):
        print "waiting for service...."
        rospy.wait_for_service('get_spines')
        print "found service!"
        try:
            get_spine_poses = rospy.ServiceProxy('get_spines', GetSpineLocations)
            response = get_spine_poses()
            spine_poses = response.spine_poses

            # debugging line
            for pose in spine_poses:
                #print pose
                pass
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        
        target_fiducial = None
        check = 0
        while target_fiducial == None and check < 100:
            # If the fiducial was not seen, try again
            rospy.sleep(0.1)
            check += 1
            for marker in self.reader.markers:
                if marker.id == target_id:
                    target_fiducial = marker

        print target_fiducial.id
        closest_pose = None
        min_dist = float('inf')
        for pose in spine_poses:
            distance = calculate_euclidean_distance(pose, target_fiducial.pose.pose)
            if distance < min_dist:
                min_dist = distance
                closest_pose = pose

        #print "Pose closest to target fiducial"
        #print closest_pose
        return closest_pose

    def grab_book(self, closest_pose):
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
        post_grasp.pose.position.x = closest_pose.position.x - (0.166 - 0.04)
        post_grasp.pose.position.y = closest_pose.position.y
        post_grasp.pose.position.z = closest_pose.position.z + 0.10

        post_grasp2 = PoseStamped()
        post_grasp2.header.frame_id = 'base_link'
        post_grasp2.pose = copy.deepcopy(closest_pose)
        post_grasp2.pose.position.x = closest_pose.position.x - (0.166 + 0.185)
        post_grasp2.pose.position.y = closest_pose.position.y
        post_grasp2.pose.position.z = closest_pose.position.z + 0.10

        #position: 
    #     x: 0.012627533637
    #     y: -0.540503621101
    #     z: 0.967533946037
    # orientation: 
    #     x: -0.736985862255
    #     y: 0.0
    #     z: 0.0
    #     w: 0.675908148289
        carry_position = PoseStamped()
        carry_position.header.frame_id = 'base_link'
        carry_position.pose.position.x = 0.012627533637
        carry_position.pose.position.y = 0.540503621101 - 0.05
        carry_position.pose.position.z = 0.967533946037
        carry_position.pose.orientation.x = -0.736985862255
        carry_position.pose.orientation.y = 0.0
        carry_position.pose.orientation.z = 0.0
        carry_position.pose.orientation.w = 0.675908148289

        # pre_grasp.pose.position.x -= (0.166 + 0.05)
        # pre_grasp.pose.orientation.w = 1

        # Note: This is only the position of the spine, not any sort of pre or post grasp
        grasp_order = ["pre_grasp", "grasp_pose", "post_grasp", "post_grasp2", "carry_position"]
        grasp_dict = {"pre_grasp": pre_grasp, "grasp_pose": grasp_pose, "post_grasp": post_grasp, "post_grasp2": post_grasp2, "carry_position": carry_position}
        for poseName in grasp_order:
            rospy.sleep(.5)
            print poseName
            err = None
            if poseName == "post_grasp" or poseName == "post_grasp2" or poseName == "carry_position":
                err = self.arm.move_to_pose(grasp_dict[poseName], num_planning_attempts=3)
            else:
                err = self.arm.move_to_pose(grasp_dict[poseName])
            if poseName == "grasp_pose":
                self.gripper.close()
                self.gripper_open = False
            if err != None:
                print "Error in move to ", poseName, " pose: ", err
                return False
        return True
        

    def remove_bounding_box(self):    
        # At the end remove collision objects
        self.planning_scene.removeCollisionObject('surface')
        

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
