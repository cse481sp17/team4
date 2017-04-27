#! /usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped

# This script is for Lab 24 (transform arithmetic) and holds util functions for transforms

def transform_to_pose(matrix):
    pose = Pose()
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]

    pose.orientation.w = np.sqrt(1 + matrix[0, 0] + matrix[1, 1] + matrix[2, 2])/2
    pose.orientation.x = (matrix[2, 1] - matrix[1, 2])/(4 * pose.orientation.w)
    pose.orientation.y = (matrix[0, 2] - matrix[2, 0])/(4 * pose.orientation.w)
    pose.orientation.z = (matrix[1, 0] - matrix[0, 1])/(4 * pose.orientation.w)

    return pose

def pose_to_transform(pose):
    matrix = np.zeros([4, 4])

    w = pose.orientation.w
    x = pose.orientation.x
    y = pose.orientation.y
    z = pose.orientation.z
    
    #rotation
    matrix[0, 0] = w**2 + x**2 - y**2 - z**2
    matrix[0, 1] = 2 * (x * y - w * z)
    matrix[0, 2] = 2 * (w * y + x * z)
    matrix[1, 0] = 2 * (x * y + w * z)
    matrix[1, 1] = w**2 - x**2 + y**2 - z**2
    matrix[1, 2] = 2 * (y * z - w * x)
    matrix[2, 0] = 2 * (x * z - w * y)
    matrix[2, 1] = 2 * (w * x + y * z)
    matrix[2, 2] = w**2 - x**2 - y**2 + z**2

    # translation
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z
    matrix[3, 3] = 1

    return matrix


def translation_and_rotation_to_transform(trans, rot):
    matrix = np.zeros([4, 4])

    w = rot[0]
    x = rot[1]
    y = rot[2]
    z = rot[3]
    
    #rotation
    matrix[0, 0] = w**2 + x**2 - y**2 - z**2
    matrix[0, 1] = 2 * (x * y - w * z)
    matrix[0, 2] = 2 * (w * y + x * z)
    matrix[1, 0] = 2 * (x * y + w * z)
    matrix[1, 1] = w**2 - x**2 + y**2 - z**2
    matrix[1, 2] = 2 * (y * z - w * x)
    matrix[2, 0] = 2 * (x * z - w * y)
    matrix[2, 1] = 2 * (w * x + y * z)
    matrix[2, 2] = w**2 - x**2 - y**2 + z**2

    # translation
    matrix[0, 3] = trans[0]
    matrix[1, 3] = trans[1]
    matrix[2, 3] = trans[2]
    matrix[3, 3] = 1

    return matrix


def wait_for_time():   
    """Wait for simulated time to begin.
    """                
    while rospy.Time().now().to_sec() == 0:                                                            
        pass     


def main():
    
    # first, create and set the pose of the object
    object_pose = PoseStamped()
    object_pose.header.frame_id = 'gripper_link'

    object_pose.pose.position.x = 0.6
    object_pose.pose.position.y = -0.1
    object_pose.pose.position.z = 0.7

    object_pose.pose.orientation.w = 0.92387953
    object_pose.pose.orientation.x = 0
    object_pose.pose.orientation.y = 0
    object_pose.pose.orientation.z = 0.38268343

    # then, calculate it as a matrix and set it to be 10 cm (0.1 m) "back" in the x-direction
    object_in_bs = pose_to_transform(object_pose.pose)
    
    # set the gripper pose relative to the object to be 0.1 meters back in the x-direction
    gripper_in_object = np.eye(4)
    gripper_in_object[0, 3] = gripper_in_object[0, 3] - 0.1

    # This above matrix is the pose of the gripper in the frame of the object.
    
    # We can multiply it by the pose of the object in the frame of the base to get
    # the pose of the gripper in the frame of the base.

    # multiply by pose of object to get result
    result_tf = np.dot(object_in_bs, gripper_in_object)
    result_pose = transform_to_pose(result_tf)
    print result_pose



if __name__ == '__main__':
    main()