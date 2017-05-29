#! /usr/bin/env python

# Read in database info to a data struct, 
# book identifier -> (BookInfo)

import util
import pickle

class DatabaseReader(object):
    def __init__(self):
        # load book data

        # uncomment htis if want to sim
        # self.library = pickle.load( open("/home/team4/catkin_ws/src/cse481c/library_bot/nodes/book_database.p", "rb") )
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

        