# This class handles the actions to move the robot to a target location (the bookcase)
# See web_nav_manager.py in /home/team4/catkin_ws/src/cse481c/map_annotator/nodes/web_nav_manager.py


import rospy
import geometry_msgs.msg
import move_base_msgs.msg
import actionlib

import pickle

from geometry_msgs.msg import Pose

class NavigationManager(object):

    def __init__(self):
        # self.currentPoseSubscriber = rospy.Subscriber("/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, callback=self.goto)# self.updateCurrentPose)
        # self.goalPublisher = rospy.Publisher("/move_base/goal", move_base_msgs.msg.MoveBaseGoal, queue_size=10)
        self.client = actionlib.SimpleActionClient('/move_base', move_base_msgs.msg.MoveBaseAction)
        #self.client.wait_for_server()

    def goto(self, goal_pose):
        destination = move_base_msgs.msg.MoveBaseGoal()
        destination.target_pose.pose = goal_pose
        destination.target_pose.header.frame_id = "map"
        self.client.send_goal(destination)
        self.client.wait_for_result()
        print "goto method ran"
    
