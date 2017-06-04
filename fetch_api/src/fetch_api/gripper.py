#! /usr/bin/env python

import actionlib
import control_msgs.msg
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

import rospy

CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
	#rospy.init_node('gripper_client')
        self.client = actionlib.SimpleActionClient('gripper_controller/gripper_action', GripperCommandAction)
        self.client.wait_for_server()
        pass

    def open(self):
        """Opens the gripper.
        """
	# create goal
        goal = control_msgs.msg.GripperCommandGoal()
	goal.command.position = OPENED_POS
	goal.command.max_effort = self.MAX_EFFORT
	
	# send goal and wait for result
	self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
	# unnecessary, but might be helpful
	if (max_effort > self.MAX_EFFORT):
		max_effort = self.MAX_EFFORT
	if (max_effort < self.MIN_EFFORT):
		max_effort = self.MIN_EFFORT
	
	# create goal
        goal = control_msgs.msg.GripperCommandGoal()
	goal.command.position = CLOSED_POS
	goal.command.max_effort = max_effort
	
	# send goal and wait for result:
	self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))

    def half_close(self, max_effort=MAX_EFFORT):
        # closes the gripper half way

        # unnecessary, but might be helpful
	if (max_effort > self.MAX_EFFORT):
		max_effort = self.MAX_EFFORT
	if (max_effort < self.MIN_EFFORT):
		max_effort = self.MIN_EFFORT
	
	# create goal
        goal = control_msgs.msg.GripperCommandGoal()
	goal.command.position = 0.05
	goal.command.max_effort = max_effort
	
	# send goal and wait for result:
	self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))

