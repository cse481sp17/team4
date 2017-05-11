# This class orchestrates all the tasks for the library bot

import database_reader
import location_driver
import perception_interpreter
import arm_controller
import Thing.srv # make this work

# TODO: fill out
def main():
    # TODO: Frontend request to retrieve book
        # TODO: make sure front end is launched
    request = None
    # TODO: read in data from database, request->book location/fiducial info
    db = DatabaseReader()
    book_info = db.request_book(request)
    # TODO: Check that we have a schematic of env, if not, build it before running this

    # TODO: Call driver to move Fetch robo to bookcase
    location_driver = LocationDriver()
    # TODO: Perceive bookcase for collision and find target book/fiducial (possibly in driver or arm)
    p_i = PerceptionInterpreter()
    # TODO: Call arm to pull out target tray and take book
    arm_control = ArmController()
    # TODO: Reset arm to noncolliding position
    arm_control.reset_position()
    # TODO: Move robot to return position w/book + release book?
    location_driver.return_to_goal()
    arm_control.release_book()

    # TODO: Send mission status to Backend->Frontend
    # publish finished message
    msg = Thing
    pass

if __name__ == '__main__':
    main()