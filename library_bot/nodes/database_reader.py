#! /usr/bin/env python

# Read in database info to a data struct, 
# book identifier -> (fiducial, book info maybe not needed, poseName) ?

import util

# TODO: fill out
class DatabaseReader(object):
    
    def __init__(self):
        # Store le db info m8
        self.stuff = dict()
        self.load_database(None)
        self.save_database(None)
        self.test_book_init()
        pass

    # stuff
    def load_database(self, file):
        pass

    # probs not needed
    def save_database(self, target):
        pass

    def request_book(self, book):
        return self.stuff[book]

    def test_book_init(self):
        book1 = (1, None, "bookshelf")
        book2 = (2, None, "return")
        if not self.stuff.__contains__(1):
            self.stuff[1] = book1
        if not self.stuff.__contains__(2):
            self.stuff[2] = book2
        