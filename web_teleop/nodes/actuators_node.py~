#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse, SetHead, SetHeadResponse, SetGripper, SetGripperResponse, SetArm, SetArmResponse

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = fetch_api.Torso()
        self._head = fetch_api.Head()
        self._gripper = fetch_api.Gripper()
        self._arm = fetch_api.Arm()

    def handle_set_torso(self, request):
        # TODO: move the torso to the requested height
        # This does what we think it does, it handles the torso height when clicked on the web interface
    # print "actuators.py, printing request: ", request
        #f = open('torsofile.txt', 'w')
    #f.write("actuators.py, printing set torso request: " + str(request.height))
    #f.close()
    #rospy.loginfo("actuators.py, printing set torso request: %.3f", request.height)
        self._torso.set_height(request.height)
    #TODO: this works when called here: self._head.pan_tilt(1,1)
        return SetTorsoResponse()

    def handle_set_head(self, request):
    #TODO move head tilt and pan
    #This works!  When called rosservice call /web_teleop/set_head <args>
    #self._head.pan_tilt(1, 1)
    #self._head.pan_tilt(request.pan, request.tilt)
    #f = open('headfile.txt', 'w')
    #f.write("actuators.py, printing set head request (tilt, pan): (" + str(request.tilt) + ", " + str(request.pan) + ")")
    #f.close()
    #rospy.loginfo("actuators.py, printing set head request (tilt, pan): (%.3f, %.3f)", (request.tilt, request.pan))
        self._head.pan_tilt(request.pan, request.tilt)
        return SetHeadResponse() # this might not work

    def handle_set_gripper(self, request):
        if request.command == -1:
            self._gripper.open()
        else:
            self._gripper.close(request.force)
        return SetGripperResponse()
        
    def handle_set_arm(self, request):
        if request.command == 0:
            #relaxed = ArmJoints.from_list([1.32, 1.40, -0.20, 1.72, 0.0, 1.66, 0.0])
            self._arm.move_to_relaxed_position()
        else:
            #extended = ArmJoints.from_list([1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0])
            self._arm.move_to_extended_position()
        return SetArmResponse()



def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso, server.handle_set_torso)
    head_service = rospy.Service('web_teleop/set_head', SetHead, server.handle_set_head)
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper, server.handle_set_gripper)
    arm_service = rospy.Service('web_teleop/set_arm', SetArm, server.handle_set_arm)
    rospy.spin()


if __name__ == '__main__':
    main()
