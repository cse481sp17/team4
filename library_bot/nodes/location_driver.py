# This class handles the actions to move the robot to a target location (the bookcase)
# See web_nav_manager.py in /home/team4/catkin_ws/src/cse481c/map_annotator/nodes/web_nav_manager.py


import rospy
import geometry_msgs.msg
import move_base_msgs.msg
import actionlib

import pickle

from geometry_msgs.msg import Pose


# class LocationDriver(object):

#     def __init__(self):
#         self.nav_manager = NavigationManager()
#         self.poses = dict()
#         self.currentPose = None
#         # self.currentPoseSubscriber = rospy.Subscriber("/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, callback=self.updateCurrentPose)
#         self.goalPublisher = rospy.Publisher("/move_base/goal", move_base_msgs.msg.MoveBaseActionGoal, queue_size=10)
#         # rospy.init_node('locDriver', anonymous=True)

#         # # create bookshelf pose
#         # bookshelfKey = "bookshelf"
#         # bookshelfPose = Pose()
#         # bookshelfPose.position.x = 2.56
#         # bookshelfPose.position.y = 3.22
#         # bookshelfPose.position.z = 0.0
#         # bookshelfPose.orientation.x = 0.0
#         # bookshelfPose.orientation.y = 0.0
#         # bookshelfPose.orientation.z = 0.016551
#         # bookshelfPose.orientation.w = 1.0

#         # self.poses[bookshelfKey] = bookshelfPose

#         # # create return pose
#         # returnKey = "return"
#         # returnPose = Pose()
#         # returnPose.position.x = 0.3548
#         # returnPose.position.y = 0.6489
#         # returnPose.position.z = 0.0
#         # returnPose.orientation.x = 0.0
#         # returnPose.orientation.y = 0.0
#         # returnPose.orientation.z = 0.14559
#         # returnPose.orientation.w = .989
#         # self.poses[returnKey] = returnPose


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
    
