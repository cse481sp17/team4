#! /usr/bin/env python

# Read in database info to a data struct, 
# book identifier -> (BookInfo)

import util
import pickle
#import main_runner

IN_SIM = False
IN_DEMO = True

class DatabaseReader(object):
    def __init__(self):
        # load book data

        #if main_runner.IN_SIM:
        if IN_SIM:
            self.library = pickle.load( open("/home/team4/catkin_ws/src/cse481c/library_bot/nodes/book_database_sim.p", "rb") )
        elif IN_DEMO:
            self.library = pickle.load( open("/home/team4/catkin_ws/src/cse481c/library_bot/nodes/book_database_DEMO.p", "rb") )
        else:
            self.library = pickle.load( open("/home/team4/catkin_ws/src/cse481c/library_bot/nodes/book_database_real_robot.p", "rb") )

    def request_book(self, bookID):
        return self.library[bookID]


class BookInfo(object):
    def __init__(self):
        self.pose = None
        self.torso_height = None
        self.head_pan = None
        self.head_tilt = None
        self.fiducial_number = None
        self.book_name = None

        