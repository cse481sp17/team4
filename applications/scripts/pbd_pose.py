import rospy
from geometry_msgs.msg import PoseStamped

class PBD_Pose(object):

    def __init__(self, pose_stamped, gripper_open):
        self.pose_stamped = pose_stamped
        self.gripper_open = gripper_open
