#! /usr/bin/env python

from geometry_msgs.msg import Twist
import rospy


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # TODO: Create publisher
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # TODO: Create Twist msg
        twistMsg = None
	twistMsg = Twist()
	twistMsg.linear.x = linear_speed
	twistMsg.linear.y = linear_speed
	twistMsg.linear.z = linear_speed
	twistMsg.angular.x = angular_speed
	twistMsg.angular.y = angular_speed
	twistMsg.angular.z = angular_speed
	# TODO: Fill out msg
        # TODO: Publish msg
	self.pub.publish(twistMsg)
        # rospy.logerr('Not implemented.')

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        twistMsg = Twist()
	self.pub.publish(twistMsg)
	#move(0, 0) ?
	# rospy.logerr('Not implemented.')
