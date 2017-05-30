#! /usr/bin/env python
import rospy

# from main_runner import BookServer
import main_runner

def main():
    rospy.init_node("libraryBotCmdlineRunner", anonymous = True)
    inpt = None
    bs = BookServer()
    bs.cmdline = True
    while True:
        if main_runner.IN_SIM:
            # Home posit already set in BookServer init
            print "Type: which bookshelf position : [1, 3]"
            # bookshelfIndxStr = raw_input()
            bookshelfIndx = int(raw_input())

            print "Type: (1=True, 0=False) whether to enable tray grabbing"
            bs.cmdline_grab_tray = int(raw_input())

            print "Type: (1=True, 0=False) whether to enable book grabbing"
            bs.cmdline_grab_book = int(raw_input())

            # bs.set_home(homeIndx)
            bs.set_bookshelf(bookshelfIndx)

            bs.book_request_callback(None)
            print "Job done"
        else: # make sure its using the right map
            print "Type: which home position: [1, 4]"
            # homeIndxStr = raw_input()
            homeIndx = int(raw_input())
            homeIndx = -homeIndx
            print "Type: which bookshelf position : [1, 4]"
            # bookshelfIndxStr = raw_input()
            bookshelfIndx = int(raw_input())

            print "Type: (1=True, 0=False) whether to enable tray grabbing"
            bs.cmdline_grab_tray = int(raw_input())

            print "Type: (1=True, 0=False) whether to enable book grabbing"
            bs.cmdline_grab_book = int(raw_input())

            bs.set_home(homeIndx)
            bs.set_bookshelf(bookshelfIndx)

            bs.book_request_callback(None)
            print "Job done"


if __name__ == '__main__':
    main()
