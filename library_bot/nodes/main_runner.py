#! /usr/bin/env python

# This class orchestrates all the tasks for the library bot

from database_reader import DatabaseReader
from location_driver import NavigationManager
from perception_interpreter import PerceptionInterpreter
from arm_controller import ArmController
from database_reader import DatabaseReader, BookInfo
from library_bot_msgs.srv import RequestBook
import rospy
import util
import fetch_api
from geometry_msgs.msg import Pose
# from library_bot import Thing.srv # make this work

# TODO: fill out
def main():
    rospy.init_node("libraryBotRunner", anonymous = True)
    
    # TODO: Frontend request to retrieve book
    # TODO: make sure front end is launched

    server = BookServer()
    book_service = rospy.Service("library_bot/book_execute", RequestBook, server.book_request_callback)
    rospy.spin()
    



class BookServer(object):
    def __init__(self):
        self.book_data = DatabaseReader()
        # self.arm_controller = ArmController()
        self.location_driver = NavigationManager()
        self.torso = fetch_api.Torso()
        self.head = fetch_api.Head()

        returnPose = Pose()
        returnPose.position.x = 0.3548
        returnPose.position.y = 0.6489
        returnPose.position.z = 0.0
        returnPose.orientation.x = 0.0
        returnPose.orientation.y = 0.0
        returnPose.orientation.z = 0.14559
        returnPose.orientation.w = .989
        self.home_pose = returnPose

    def book_request_callback(self, data):
        # get book information
        bookID = data.book_id
        book_info = self.book_data.library[bookID]
        print book_info.pose
        target_id = book_info.fiducial_number

        # navigate to book
        self.location_driver.goto(book_info.pose)

        # move torso
        #self.torso.setHeight(book_info.torso_height)

        # move head (pan and tilt)
        #self.head.setPanTilt(book_info.head_pan, book_info.head_tilt)

        # # execute grasping procedure
        # self.arm_controller.add_bounding_box()
        # self.arm_controller.grab_tray(target_id)
        # closest_pose = self.arm_controller.find_grasp_pose(target_id)
        # self.arm_controller.grab_book(closest_pose)
        # self.arm_controller.remove_bounding_box()

        # # move head (pan and tilt)
        # self.head.setPanTilt(0.0, 0.0)

        # # move torso
        # self.torso.setHeight(0.0)
        
        # # navigate back home
        # self.location_driver.goto(self.home_pose)

        # # drop book somewhere
        # self.arm_controller.open_gripper()

        # TODO: set response?
        #return BookRequestCallbackResponse()


if __name__ == '__main__':
    main()



# TODO: Check that we have a schematic of env, if not, build it before running this

# For now, these poses are saved manually, thru cmd_line_navigation.py + keyboard_teleop
# TODO: might need to add to our launch folder: "roslaunch applications nav_rviz.launch"

# Subscribe to move_base/status and wait until status switches from "goal accepted" to "goal reached" before continuing actions

# TODO: Perceive bookcase for collision and find target book/fiducial (possibly in driver or arm)
# p_i = PerceptionInterpreter()
# TODO: Call arm to pull out target tray and take book
# arm_control = ArmController()
# TODO: Reset arm to noncolliding position
# arm_control.reset_position()
# TODO: Move robot to return position w/book + release book?
# location_driver.return_to_goal()
# arm_control.release_book()

# TODO: Send mission status to Backend->Frontend
# publish finished message
# msg = Thing

# rospy.sleep(2)
# location_driver.goto("return")