#! /usr/bin/env python

import rospy
import fetch_api
from fetch_api import Arm
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped



def main():
    # ... init ...
    rospy.init_node("cart_arm_demo")
    arm = Arm()
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)
   
   # Move the arm
    # pose1 = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
    # pose2 = Pose(Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))
    # small changes to test book spine grabbing
    pose1 = Pose(Point((0.488130 - 0.15), -0.311928, 0.831613), Quaternion(0.0, 0.0, 0.0, 1))
    
    
    ps1 = PoseStamped()
    ps1.header.frame_id = 'base_link'
    ps1.pose = pose1
    #ps2 = PoseStamped()
    #ps2.header.frame_id = 'base_link'
    #rosrunps2.pose = pose2
    #gripper_poses = [ps1, ps2]
    gripper_poses = [ps1]

    while (1):
        for pose in gripper_poses:
            error = arm.move_to_pose(pose)
            if error != None:
                rospy.logerr(error)
            rospy.sleep(1)


if __name__ == '__main__':
    main()
