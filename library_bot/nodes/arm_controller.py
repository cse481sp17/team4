import fetch_api
import pickle

# Calls arm to pull out target tray and take book

class ArmController(object):

    def __init__(self):
        # set arm and gripper, and fiducial id
        self.arm = fetch_api.Arm()
        self.gripper = fetch_api.Gripper()
        # Will need to change this to something real at some point
        self.target_id = None# target_id

        # First, load the poses for the fiducial insert movement
        self.sequence = pickle.load( open(INSERT_GRASP_POSES, "rb") )
        self.gripper_open = True

        # # Init the reader for the tags
        self.reader = ArTagReader()
        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)

    def open_gripper(self):
        self.gripper.open()
        self.gripper_open = True

    def add_bounding_box(self):
        # Init the planning scene for collisions
        planning_scene = PlanningSceneInterface('base_link')

        print "waiting for service...."
        rospy.wait_for_service('get_spines')
        print "found service!"

        get_spine_poses = rospy.ServiceProxy('get_spines', GetSpineLocations)
        response = get_spine_poses()
        #planning_scene.addBox('surface', response.surface_x_size, response.surface_y_size, response.surface_z_size,
        #    response.surface_pose.position.x, response.surface_pose.position.y, response.surface_pose.position.z)
    

    def grab_tray(self, target_id):
        # This is the same as the pbd action stuff, not making any changes at the moment
        for pbd_pose in self.sequence:
            move_pose = PoseStamped()
            move_pose.header.frame_id = 'base_link'
            if pbd_pose.frame == 'base_link':
                move_pose.pose = pbd_pose.pose
            else:
                for marker in self.reader.markers:
                    if target_id == marker.id:
                        print "Calculating pose relative to marker...."

                        # Transform the pose to be in the base_link frame
                        pose_in_tag_frame = pose_to_transform(pbd_pose.pose)
                        tag_in_base_frame = pose_to_transform(marker.pose.pose)

                        target_matrix = np.dot(tag_in_base_frame, pose_in_tag_frame)

                        target_pose = transform_to_pose(target_matrix)

                        move_pose.pose = target_pose

            err = self.arm.move_to_pose(move_pose)
            print "Error in move to pose: ", err
            # Check the gripper to open/close
            if pbd_pose.gripper_open != self.gripper_open:
                if self.gripper_open == True:
                    self.gripper.close()
                    self.gripper_open = False
                else:
                    self.gripper.open()
                    self.gripper_open = True

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
                print pose
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        
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
        return closest_pose

    def grab_book(self, closest_pose):
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.pose = closest_pose
        # Offset because the arm is moved relative to the wrist roll Joint
        grasp_pose.pose.position.x -= 0.166
        grasp_pose.pose.orientation.w = 1

        # Note: This is only the position of the spine, not any sort of pre or post grasp
        err = self.arm.move_to_pose(grasp_pose)
        print "Error in move to pose: ", err

    def remove_bounding_box(self):    
        # At the end remove collision objects
        planning_scene.removeCollisionObject('surface')
        

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
