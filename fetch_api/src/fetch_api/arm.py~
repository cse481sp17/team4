import actionlib
import control_msgs.msg
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
        # create actionlib client and wait for server
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
        
        # create a TrajectoryPoint
        trajectoryPoint = trajectory_msgs.msg.JointTrajectoryPoint()
        trajectoryPoint.positions = joint_values
        trajectoryPoint.time_from_start = rospy.Duration.from_sec(5.0)
        
        # create a JointTrajectory
        jointTrajectory = trajectory_msgs.msg.JointTrajectory()
        jointTrajectory.joint_names = joint_names
        jointTrajectory.points.append(trajectoryPoint)
             
        # create and send goal        
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory = jointTrajectory
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
    
    def move_to_relaxed_position(self):
        relaxed = ArmJoints.from_list([1.32, 1.40, -0.20, 1.72, 0.0, 1.66, 0.0])
        self.move_to_joints(relaxed)

    def move_to_extended_position(self):
        extended = ArmJoints.from_list([-0.496, 0.311, -1.31, -0.63, -2.95, 0.181, -2.14])
        self.move_to_joints(extended)

