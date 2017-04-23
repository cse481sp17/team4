import rospy
import geometry_msgs.msg
import move_base_msgs.msg

import pickle

class NavigationManager(object):

    def __init__(self):
        self.poses = dict()
        self.currentPose = None
        self.currentPoseSubscriber = rospy.Subscriber("/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, callback=self.updateCurrentPose)
        self.goalPublisher = rospy.Publisher("/move_base/goal", move_base_msgs.msg.MoveBaseActionGoal, queue_size=10)
        rospy.init_node('navManager', anonymous=True)

    def updateCurrentPose(self, data):
        self.currentPose = data.pose.pose
    
    def loadPoses(self):
        self.poses = pickle.load( open("save.p", "rb") )

    def listPoses(self):
        return self.poses.keys()
    
    def savePose(self, poseName):
        self.poses[poseName] = self.currentPose
        # update persistent information
        pickle.dump( self.poses, open("save.p", "wb") )

    def deletePose(self, poseName):
        del self.poses[poseName]
        # update persistent information
        pickle.dump( self.poses, open("save.p", "wb") )

    def goto(self, poseName):
        destination = move_base_msgs.msg.MoveBaseActionGoal()
        destination.goal.target_pose.pose = self.poses[poseName]
        destination.goal.target_pose.header.frame_id = "map"
        self.goalPublisher.publish(destination)
    
