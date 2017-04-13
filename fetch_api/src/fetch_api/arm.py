# TODO: import ?????????
import actionlib
# TODO: import ???????_msgs.msg
import control_msgs.msg
# TODO: import ??????????_msgs.msg
import trajectory_msgs.msg
import rospy

from .arm_joints import ArmJoints


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
	    self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
	    self.client.wait_for_server()
    pass

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        joint_names = arm_joints.names()
        joint_values = arm_joints.values()

        # TODO: Create a trajectory point
        # TODO: Set position of trajectory point
        # TODO: Set time of trajectory point
        trajectoryPoint = trajectory_msgs.msg.JointTrajectoryPoint()
        trajectoryPoint.positions = joint_values
        trajectoryPoint.time_from_start = rospy.Duration.from_sec(5.0)


        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory
        jointTrajectory = trajectory_msgs.msg.JointTrajectory()
        jointTrajectory.joint_names = joint_names
        jointTrajectory.points.append(trajectoryPoint)
        
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory = jointTrajectory
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))


        # TODO: Send goal
        # TODO: Wait for result
        #rospy.logerr('Not implemented.')
