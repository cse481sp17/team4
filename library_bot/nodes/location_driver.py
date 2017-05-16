# This class handles the actions to move the robot to a target location (the bookcase)

import rospy
import geometry_msgs.msg
import move_base_msgs.msg

from geometry_msgs.msg import Pose


class LocationDriver(object):

    def __init__(self):
        self.poses = dict()
        self.currentPose = None
        self.currentPoseSubscriber = rospy.Subscriber("/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, callback=self.updateCurrentPose)
        self.goalPublisher = rospy.Publisher("/move_base/goal", move_base_msgs.msg.MoveBaseActionGoal, queue_size=10)
        # rospy.init_node('locDriver', anonymous=True)

        # create bookshelf pose
        bookshelfPose = Pose()
        bookshelfPose.position.x = 2.56
        bookshelfPose.position.y = 3.22
        bookshelfPose.position.z = 0.0
        bookshelfPose.orientation.x = 0.0
        bookshelfPose.orientation.y = 0.0
        bookshelfPose.orientation.z = 0.016551
        bookshelfPose.orientation.w = 1.0

        self.poses["bookshelf"] = bookshelfPose

        # create return pose

        returnPose = Pose()
        returnPose.position.x = 0.3548
        returnPose.position.y = 0.6489
        returnPose.position.z = 0.0
        returnPose.orientation.x = 0.0
        returnPose.orientation.y = 0.0
        returnPose.orientation.z = 0.14559
        returnPose.orientation.w = .989
        self.poses["return"] = returnPose

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

    # parameter: location is a string with the name of the target pose
    def move_to_location(self, location):
        destination = move_base_msgs.msg.MoveBaseActionGoal()
        destination.goal.target_pose.pose = self.poses[location]
        destination.goal.target_pose.header.frame_id = "map"
        self.goalPublisher.publish(destination)
        print "here1"

    def return_to_goal(self, goal):
        destination = move_base_msgs.msg.MoveBaseActionGoal()
        destination.goal.target_pose.pose = self.poses[goal]
        destination.goal.target_pose.header.frame_id = "map"
        self.goalPublisher.publish(destination)
        print "here2"
