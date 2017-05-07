#! /usr/bin/env python

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from ar_track_alvar_msgs.msg import AlvarMarkers
import fetch_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers


def main():
    rospy.init_node('hallucinogen')
    # wait_for_time()

    start = PoseStamped()
    start.header.frame_id = 'base_link'
    start.pose.position.x = 0.5
    start.pose.position.y = 0.5
    start.pose.position.z = 0.75
    arm = fetch_api.Arm()
    arm.move_to_pose(start)
                                                                               
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback) # Subscribe to AR tag poses, use reader.callback
    
    while len(reader.markers) == 0:
        rospy.sleep(0.1)
    
    for marker in reader.markers:
        # get the pose to move to
        poseStamp = PoseStamped()
        poseStamp.header.frame_id = 'base_link'
        poseStamp.pose = marker.pose.pose
        error = arm.move_to_pose(poseStamp)
        if error is None:
            rospy.loginfo('Moved to marker {}'.format(marker.id))
            return
        else:
            rospy.logwarn('Failed to move to marker {}'.format(marker.id))
    rospy.logerr('Failed to move to any markers!')


if __name__ == '__main__':
    main()