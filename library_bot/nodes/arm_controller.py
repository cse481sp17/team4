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
        response = get_spine_poses() # 0.17
        self.planning_scene.addBox('surface', (response.surface_x_size - 0.19), response.surface_y_size, response.surface_z_size,
           response.surface_pose.position.x, response.surface_pose.position.y, response.surface_pose.position.z)


    # Adding a bounding box for the delivery table
    def add_delivery_bounding_box(self):
        print "waiting for service...."
        rospy.wait_for_service('get_spines')
        print "found service!"

        get_spine_poses = rospy.ServiceProxy('get_spines', GetSpineLocations)
        response = get_spine_poses()
        self.planning_scene.addBox('surface', (response.surface_x_size + 0.05), response.surface_y_size, response.surface_z_size,
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
            print "Fiducial not found :("
        else:
            print "Fiducial Found :)"
            #print target_marker_pose

        target_marker_pose.orientation.x = 0.0
        target_marker_pose.orientation.y = 0.0
        target_marker_pose.orientation.z = 0.0
        target_marker_pose.orientation.w = 1.0


        everError = None

        # This is the same as the pbd action stuff, not making any changes at the moment
        for pbd_pose in self.sequence:
            move_pose = PoseStamped()
            move_pose.header.frame_id = 'base_link'
            if pbd_pose.frame == 'base_link':
                move_pose.pose = pbd_pose.pose
                print "Default pose"
                print pbd_pose.pose
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

            #rospy.sleep(1)
            err = self.arm.move_to_pose(move_pose, num_planning_attempts=2) # 3
            print "Error in move to pose: ", err
            temp = 0
            while temp < 3 and err != None:
                #if err != None:
                #return False
                print "Trying This pose again"
                err = self.arm.move_to_pose(move_pose, num_planning_attempts=3, tolerance=0.03)
                print "Error in move to pose: ", err
                temp += 1
            if err != None:
                print "Arm failed to move to pose"

                print "Returning to Default Pose"
                default_pose = PoseStamped()
                default_pose.header.frame_id = 'base_link'
                default_pose.pose.position.x = 0.0
                default_pose.pose.position.y = 0.467635764991
                default_pose.pose.position.z = 0.743876436337
                default_pose.pose.orientation.x = 0.0
                default_pose.pose.orientation.y = 0.0
                default_pose.pose.orientation.z = 0.0
                default_pose.pose.orientation.w = 1.0

                err = self.arm.move_to_pose(default_pose)
                print "returned to default pose"
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
        target_fiducial = None
        check = 0
        while target_fiducial == None and check < 100:
            # If the fiducial was not seen, try again
            rospy.sleep(0.1)
            check += 1
            for marker in self.reader.markers:
                if marker.id == target_id:
                    target_fiducial = marker

        if target_fiducial == None:
            print "Failed to find fiducial :("
        else:
            print "Found fidcuail!"


        check = 0
        found_good_pose = False
        closest_pose = None
        print "waiting for service...."
        rospy.wait_for_service('get_spines')
        print "found service!"
        try:
            while check < 20 and found_good_pose == False:

                get_spine_poses = rospy.ServiceProxy('get_spines', GetSpineLocations)
                response = get_spine_poses()
                spine_poses = response.spine_poses

                min_dist = float('inf')
                for pose in spine_poses:
                    distance = calculate_euclidean_distance(pose, target_fiducial.pose.pose)
                    if distance < min_dist:
                        min_dist = distance
                        closest_pose = pose
                check += 1
                if closest_pose.position.x > target_fiducial.pose.pose.position.x and closest_pose.position.y < (target_fiducial.pose.pose.position.y + 0.025) and closest_pose.position.y > (target_fiducial.pose.pose.position.y - 0.025):
                    found_good_pose = True
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        # print target_fiducial.id
        # closest_pose = None
        # min_dist = float('inf')
        # for pose in spine_poses:
        #     distance = calculate_euclidean_distance(pose, target_fiducial.pose.pose)
        #     if distance < min_dist:
        #         min_dist = distance
        #         closest_pose = pose

        if found_good_pose == False:
            print "Failed to find good pose"
        else:
            print "Found good pose"

        #print "Pose closest to target fiducial"
        #print closest_pose
        return closest_pose

    def grab_book(self, closest_pose):
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.pose = copy.deepcopy(closest_pose)
        # Offset because the arm is moved relative to the wrist roll Joint
        grasp_pose.pose.position.x -= (0.166 - 0.04) # 0.035
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
        post_grasp.pose.position.z = closest_pose.position.z + 0.12 # 0.10

        post_grasp2 = PoseStamped()
        post_grasp2.header.frame_id = 'base_link'
        post_grasp2.pose = copy.deepcopy(closest_pose)
        post_grasp2.pose.position.x = closest_pose.position.x - (0.166 + 0.185)
        post_grasp2.pose.position.y = closest_pose.position.y
        post_grasp2.pose.position.z = closest_pose.position.z + 0.12 # 0.10

        #position: 
    #     x: 0.012627533637
    #     y: -0.540503621101
    #     z: 0.967533946037
    # orientation: 
    #     x: -0.736985862255
    #     y: 0.0
    #     z: 0.0
    #     w: 0.675908148289
        # carry_position = PoseStamped()
        # carry_position.header.frame_id = 'base_link'
        # carry_position.pose.position.x = 0.012627533637
        # carry_position.pose.position.y = 0.540503621101 - 0.05
        # carry_position.pose.position.z = 0.967533946037
        # carry_position.pose.orientation.x = -0.736985862255
        # carry_position.pose.orientation.y = 0.0
        # carry_position.pose.orientation.z = 0.0
        # carry_position.pose.orientation.w = 0.675908148289


        # new carry pose
        carry_position = PoseStamped()
        carry_position.header.frame_id = 'base_link'
        carry_position.pose.position.x = -0.0567592047155
        carry_position.pose.position.y = 0.356767743826
        carry_position.pose.position.z = 0.865676641464
        carry_position.pose.orientation.x = -0.706504225731
        carry_position.pose.orientation.y = 0.0401065722108
        carry_position.pose.orientation.z = -0.705501019955
        carry_position.pose.orientation.w = -0.0388784334064

        # pre_grasp.pose.position.x -= (0.166 + 0.05)
        # pre_grasp.pose.orientation.w = 1

        # Note: This is only the position of the spine, not any sort of pre or post grasp
        grasp_order = ["pre_grasp", "grasp_pose", "post_grasp", "post_grasp2", "carry_position"]
        grasp_dict = {"pre_grasp": pre_grasp, "grasp_pose": grasp_pose, "post_grasp": post_grasp, "post_grasp2": post_grasp2, "carry_position": carry_position}
        for poseName in grasp_order:
            #rospy.sleep(.5)
            print poseName
            err = None
            if poseName == "post_grasp" or poseName == "post_grasp2" or poseName == "carry_position":
                err = self.arm.move_to_pose(grasp_dict[poseName], num_planning_attempts=3, replan=True)
            else:
                err = self.arm.move_to_pose(grasp_dict[poseName], num_planning_attempts=3, replan=True)
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


    def delivery(self):

        # delivery_pose = PoseStamped()
        # delivery_pose.header.frame_id = "base_link"
        # delivery_pose.pose.position.x = 0.641032576561
        # delivery_pose.pose.position.y = 0.0756497383118
        # delivery_pose.pose.position.z = 0.36987259984
        # delivery_pose.pose.orientation.x = -0.689870417118
        # delivery_pose.pose.orientation.y = 0.135909467936
        # delivery_pose.pose.orientation.z = 0.137442305684
        # delivery_pose.pose.orientation.w = 0.697651088238

        # new Delivery pose

        # delivery_pose = PoseStamped()
        # delivery_pose.header.frame_id = "base_link"
        # delivery_pose.pose.position.x = 0.605564773083
        # delivery_pose.pose.position.y = 0.0
        # delivery_pose.pose.position.z = 0.902032971382
        # delivery_pose.pose.orientation.x = -0.708695232868
        # delivery_pose.pose.orientation.y = 0.054994776845
        # delivery_pose.pose.orientation.z = 0.054417796433
        # delivery_pose.pose.orientation.w = 0.70125991106

        # Once again, new poses (saved with respect to torso 0.4 on the robot)

        delivery_pose = PoseStamped()
        delivery_pose.header.frame_id = "base_link"
        delivery_pose.pose.position.x = 0.67331725359
        delivery_pose.pose.position.y = 0.0
        delivery_pose.pose.position.z = 0.962293148041
        delivery_pose.pose.orientation.x = 0.686241090298
        delivery_pose.pose.orientation.y = 0.135450080037
        delivery_pose.pose.orientation.z = -0.138387724757
        delivery_pose.pose.orientation.w = 0.701124310493


        #err = self.arm.move_to_pose(delivery_pose, num_planning_attempts=3, replan=True)
        err = self.arm.move_to_pose(delivery_pose)

        print "Error in move to  delivery pose: ", err
        check = 0
        while err != None and check < 3:
            err = self.arm.move_to_pose(delivery_pose, num_planning_attempts=3, replan=True)
            print "Error in move to  delivery pose: ", err
            check += 1



    def curl_arm(self):

        # curled_pose = PoseStamped()
        # curled_pose.header.frame_id = "base_link"
        # curled_pose.pose.position.x = -0.00938361883163
        # curled_pose.pose.position.y = 0.38768684864
        # curled_pose.pose.position.z = 0.742396354675
        # curled_pose.pose.orientation.x = -0.999579787254
        # curled_pose.pose.orientation.y = -0.0128491902724
        # curled_pose.pose.orientation.z = 0.00204527354799
        # curled_pose.pose.orientation.w = -0.0259021110833

        # New curled pose

        # curled_pose = PoseStamped()
        # curled_pose.header.frame_id = "base_link"
        # curled_pose.pose.position.x = -0.10520786792
        # curled_pose.pose.position.y = 0.275289952755
        # curled_pose.pose.position.z = 0.855269372463 + 0.05
        # curled_pose.pose.orientation.x = -0.510465681553
        # curled_pose.pose.orientation.y = -0.489717870951
        # curled_pose.pose.orientation.z = -0.524574935436
        # curled_pose.pose.orientation.w = 0.473732382059

        # Once again, new poses (saved with respect to a torso height of 4)

        curled_pose = PoseStamped()
        curled_pose.header.frame_id = "base_link"
        curled_pose.pose.position.x = 0.0904720053077
        curled_pose.pose.position.y = 0.146827042103
        curled_pose.pose.position.z = 0.939861655235
        curled_pose.pose.orientation.x = 0.49698728323
        curled_pose.pose.orientation.y = -0.492216616869
        curled_pose.pose.orientation.z = 0.502891778946
        curled_pose.pose.orientation.w = 0.507765948772
        

        #err = self.arm.move_to_pose(curled_pose, num_planning_attempts=3, replan=True)
        err = self.arm.move_to_pose(curled_pose)

        print "Error in move to curled pose: ", err

        check = 0
        while err != None and check < 3:
            err = self.arm.move_to_pose(curled_pose, num_planning_attempts=3, replan=True)
            print "Error in move to  delivery pose: ", err
            check += 1
        

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
