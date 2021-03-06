#!/usr/bin/env python

import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

ACTION_NAME = control_msgs.msg.FollowJointTrajectoryAction
JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
	self.client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', ACTION_NAME)
	self.client.wait_for_server()
        pass

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        
	if height > self.MAX_HEIGHT:
		height = self.MAX_HEIGHT
	if height < self.MIN_HEIGHT:
		height = self.MIN_HEIGHT
	
        
	trajectoryPoint = trajectory_msgs.msg.JointTrajectoryPoint()
        
	trajectoryPoint.positions = [height]
	
        
	trajectoryPoint.time_from_start = rospy.Duration.from_sec(TIME_FROM_START)

	jointTrajectory = trajectory_msgs.msg.JointTrajectory()
	jointTrajectory.joint_names.append(JOINT_NAME)
	jointTrajectory.points.append(trajectoryPoint)
        
	goal = control_msgs.msg.FollowJointTrajectoryGoal()
        
        
	goal.trajectory = jointTrajectory
        
	self.client.send_goal(goal)
        
	self.client.wait_for_result(rospy.Duration.from_sec(5.0))

