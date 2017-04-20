#! /usr/bin/env python

from geometry_msgs.msg import Twist
import rospy
import nav_msgs.msg
import math
import tf.transformations as tft
import numpy as np
import copy


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.currentOdom = nav_msgs.msg.Odometry()
        self._odom_sub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, callback=self._odom_callback)

    def _odom_callback(self, msg):
        self.currentOdom = msg

	

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
        twistMsg = None
        twistMsg = Twist()
        twistMsg.linear.x = linear_speed
        twistMsg.linear.y = linear_speed
        twistMsg.linear.z = linear_speed
        twistMsg.angular.x = angular_speed
        twistMsg.angular.y = angular_speed
        twistMsg.angular.z = angular_speed
        self.pub.publish(twistMsg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        twistMsg = Twist()
        self.pub.publish(twistMsg)


    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while (self.currentOdom == None):
            rospy.sleep(rospy.Duration.from_sec(0.005))
        

        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self.currentOdom)
        startPoint = start.pose.pose.position
        rate = rospy.Rate(10)

        position = self.currentOdom.pose.pose.position

        # TODO: CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        while distanceBetweenPoints(position, startPoint) < abs(distance):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            position = self.currentOdom.pose.pose.position
            rate.sleep()


    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        while (self.currentOdom == None):
            rospy.sleep(rospy.Duration.from_sec(0.005))
        start = copy.deepcopy(self.currentOdom)
        startOrient = start.pose.pose.orientation
        startAngle = quaternion_to_yaw(startOrient)
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        while angular_distance > 2 * math.pi:
            angular_distance -= 2 * math.pi
        while angular_distance < -2 * math.pi:
            angular_distance += 2 * math.pi

        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        orientation = self.currentOdom.pose.pose.orientation
        angle = quaternion_to_yaw(orientation)

        travelDistance = abs(startAngle - angle)
        # TODO: Be sure to handle the case where the desired amount is negative!
        while travelDistance < abs(angular_distance):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            orientation = self.currentOdom.pose.pose.orientation
            angle = quaternion_to_yaw(orientation)
            travelDistance = abs(startAngle - angle)
            rate.sleep()



def distanceBetweenPoints(point1, point2):
    distance = (point1.x - point2.x)**2 + (point1.y - point2.y)**2
    return math.sqrt(distance)

def quaternion_to_yaw(q):
    m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    x = m[0, 0]
    y = m[1, 0]
    return math.atan2(y, x)