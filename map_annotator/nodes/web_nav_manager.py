import rospy
import geometry_msgs.msg
import move_base_msgs.msg

import pickle

class WebNavigationManager(object):

    def __init__(self):
        self.poses = dict()
        self.currentPose = None
        self.goalPublisher = rospy.Publisher("/move_base/goal", move_base_msgs.msg.MoveBaseActionGoal, queue_size=10)
        #rospy.init_node('webNavManager', anonymous=True)
    
    def loadPoses(self):
        self.poses = pickle.load( open("save.p", "rb") )

    def listPoses(self):
        return self.poses.keys()
    
    def savePose(self, poseName, pose):
        self.poses[poseName] = pose
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
        #return destination
        self.goalPublisher.publish(destination)
    
