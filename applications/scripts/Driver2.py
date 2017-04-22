import copy
import math
import rospy
import tf.transformations as tft
import numpy as np

class Driver(object):
    ANGULAR_SPEED = 0.5
    SPEED = 0.1

    def __init__(self, base):
         self.goal = None
         self._base = base

    def start(self):
        state = 'turn'
        goal = None
        desired_distance = 0.0
        while True:
            # Check if someone changed self.goal from outside
            if goal != self.goal:
                goal = copy.deepcopy(self.goal)
                startPos = copy.deepcopy(self._base.currentOdom.pose.pose)
                desired_distance = distanceBetweenPoints(goal, self._base.currentOdom.pose.pose.position) # Set this to how far the robot should move once pointed in the right direction
                state = 'turn'
            
            # current position and angle
            currentPoint = self._base.currentOdom.pose.pose.position
            currentAngle = quaternion_to_yaw(self._base.currentOdom.pose.pose.orientation)

            if state == 'turn' and goal != None:
                # desired angle
                offsetX = goal.x - currentPoint.x 
                offsetY = goal.y - currentPoint.y
                targetAngle = math.atan2(offsetY, offsetX)

                # angle to turn
                angularDistance = targetAngle - currentAngle

                if angularDistance > math.pi:
                    angularDistance = angularDistance - 2 * math.pi

                # if the angle to turn is greater than 5 degrees (threshold), 
                if abs(angularDistance) > 5 * math.pi/180:
                    direction = -1 if angularDistance < 0 else 1
                    angularSpeed = max(0.25, min(1, abs(angularDistance)))                  
                    self._base.move(0, direction * angularSpeed)
                else:
                    state = 'move'

            if state == 'move':
                # calculate current position and distance left to go
                forwardDistance = distanceBetweenPoints(currentPoint, goal)

                # calculate whether direction is forwards or backwards
                offsetX = goal.x - currentPoint.x 
                offsetY = goal.y - currentPoint.y
                targetAngle = math.atan2(offsetY, offsetX)

                # check for overshoot
                direction = 1
                if abs(targetAngle - math.pi) < 5 * math.pi/180:
                    direction = -1

                # if the distance left to move is greater than 0.01, keep moving
                if (forwardDistance > 0.1):
                    linearSpeed = max(0.05, min(0.5, forwardDistance))
                    #direction = -1 if desired_distance < 0 else 1
                    self._base.move(direction * linearSpeed, 0)

                # check again to see if angle needs to be corrected
                if abs(targetAngle - currentAngle) > 5 * math.pi/180:
                    state = 'turn'

            rospy.sleep(0.1)


def distanceBetweenPoints(point1, point2):
    distance = (point1.x - point2.x)**2 + (point1.y - point2.y)**2
    return math.sqrt(distance)

def quaternion_to_yaw(q):
    m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    x = m[0, 0]
    y = m[1, 0]
    return math.atan2(y, x)