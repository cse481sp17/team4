import copy
import math
import rospy
import tf.transformations as tft
import numpy as np

class Driver(object):
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

            if state == 'turn' and goal != None:
                # TODO: Fix Rotation. May need to work on threshold
                currentPoint = self._base.currentOdom.pose.pose.position
                offsetX = goal.x - currentPoint.x 
                offsetY = goal.y - currentPoint.y
                targetAngle = math.atan2(offsetY, offsetX)
                currentAngle = quaternion_to_yaw(self._base.currentOdom.pose.pose.orientation)
                angularDistance = currentAngle - targetAngle
                travelDistance = quaternion_to_yaw(startPos.orientation) - currentAngle
                STILL_NEED_TO_TURN = travelDistance < abs(angularDistance)
                #TODO: Handle Overshooting
                if STILL_NEED_TO_TURN:
                    direction = -1 if angularDistance < 0 else 1                    
                    self._base.move(0, direction * 0.5)
                else:
                    state = 'move'

            if state == 'move':
                position = self._base.currentOdom.pose.pose.position
                STILL_NEED_TO_MOVE = (distanceBetweenPoints(position, startPos.position) < abs(desired_distance))
                #TODO: Handle overshooting
                if STILL_NEED_TO_MOVE:
                    # TODO: possibly adjust speed to slow down when close to the goal
                    direction = -1 if desired_distance < 0 else 1
                    self._base.move(direction * 0.1, 0)

            rospy.sleep(0.1)


def distanceBetweenPoints(point1, point2):
    distance = (point1.x - point2.x)**2 + (point1.y - point2.y)**2
    return math.sqrt(distance)

def quaternion_to_yaw(q):
    m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    x = m[0, 0]
    y = m[1, 0]
    return math.atan2(y, x)