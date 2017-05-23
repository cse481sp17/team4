#! /usr/bin/env python

# Read in database info to a data struct, 
# book identifier -> (BookInfo)

import util
import pickle


class DatabaseReader(object):
    def __init__(self):
        # load book data
        self.library = pickle.load( open("bookData.p", "rb") )

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

        