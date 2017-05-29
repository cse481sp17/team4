import rospy
from geometry_msgs.msg import Pose

class PBD_Pose(object):

    def __init__(self, pose, gripper_open, frame):
        self.pose = pose
        self.gripper_open = gripper_open
        self.frame = frame
