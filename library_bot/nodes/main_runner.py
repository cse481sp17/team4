#! /usr/bin/env python

# This class orchestrates all the tasks for the library bot

from database_reader import DatabaseReader
from location_driver import NavigationManager
from perception_interpreter import PerceptionInterpreter
from arm_controller import ArmController
from database_reader import DatabaseReader, BookInfo
from library_bot_msgs.srv import RequestBook, RequestBookResponse
import rospy
import util
import fetch_api
from geometry_msgs.msg import Pose
# from library_bot import Thing.srv # make this work

from visualization_msgs.msg import Marker

IN_SIM = False  # fo evythin bby

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
        self.arm_controller = ArmController()
        self.location_driver = NavigationManager()
        self.torso = fetch_api.Torso()
        self.head = fetch_api.Head()
        self.cmdline = False
        self.cmdline_grab_tray = False
        self.cmdline_grab_book = False

        # home for sim.  could be refactored better
        # returnPose = Pose()
        # returnPose.position.x = 0.3548
        # returnPose.position.y = 0.6489
        # returnPose.position.z = 0.0
        # returnPose.orientation.x = 0.0
        # returnPose.orientation.y = 0.0
        # returnPose.orientation.z = 0.14559
        # returnPose.orientation.w = .989

        # home for real robot as negative book indices
        # self.home_pose = returnPose
        self.home_pose = self.book_data.library[-1].pose
        self.delivery_pose = self.book_data.library[-2].pose
   
    # Call this before doing the callback from cmdline
    def set_home(self, homeIndx):
        self.home_pose = self.book_data.library[homeIndx].pose

    # Call this before doing the callback from cmdline
    def set_bookshelf(self, bookshelfIndx):
        self.bookshelfIndex = bookshelfIndx
        self.bookshelf = self.book_data.library[bookshelfIndx].pose

    def book_request_callback(self, data):
        # get book information
        bookID = None
        if self.cmdline:
            bookID = self.bookshelfIndex
        else:
            bookID = data.book_id

        book_info = self.book_data.library[bookID]
        print book_info.pose
        target_id = book_info.fiducial_number

        # Go home before anywhere else
        self.location_driver.goto(self.home_pose)

        # Temp code to publish the shelf location
        marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        rospy.sleep(0.5)
        box_marker = Marker()
        box_marker.type = Marker.ARROW
        box_marker.header.frame_id = "map"
        box_marker.pose = book_info.pose
        box_marker.scale.x = 0.5
        box_marker.scale.y = 0.1
        box_marker.scale.z = 0.1
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0
        marker_pub.publish(box_marker)

        # navigate to book
        self.location_driver.goto(book_info.pose)

        # move torso
        self.torso.set_height(book_info.torso_height)

        # move head (pan and tilt)
        self.head.pan_tilt(book_info.head_pan, book_info.head_tilt)

        # execute grasping procedure
        self.arm_controller.add_bounding_box()
        
        # t/f if grab tray
        grab_tray_success = None
        closest_pose = None
        grab_book_success = None

        if not self.cmdline or (self.cmdline and self.cmdline_grab_tray):
            grab_tray_success = self.arm_controller.grab_tray(target_id)
            closest_pose = self.arm_controller.find_grasp_pose(target_id)
        # t/f if grab book
        if not self.cmdline or (self.cmdline and self.cmdline_grab_book):
            grab_book_success = self.arm_controller.grab_book(closest_pose)
            self.arm_controller.remove_bounding_box()

        # move head (pan and tilt)
        self.head.pan_tilt(0.0, 0.0)

        # move torso
        self.torso.set_height(0.0)

        # self.location_driver.goto(self.delivery_pose)
        # self.arm_controller.delivery()
        # self.arm_controller.open_gripper()
        # self.arm_controller.curl_arm()
        
        # navigate back home
        self.location_driver.goto(self.home_pose)

        # TODO: set response?

        if not self.cmdline:
            success_msg = RequestBookResponse()
            success_msg.book_id_response = data.book_id
            success_msg.success = int(True)# int(grab_book_success and grab_tray_success)

            return success_msg


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