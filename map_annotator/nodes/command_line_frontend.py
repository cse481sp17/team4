import rospy
#import geometry_msgs.msg
#import move_base_msgs.msg
from backend import NavigationManager



def list(nav):
    if not nav.poses:
        print "No poses :("
    else:
        print "Poses:"
        poseList = nav.listPoses()
        for pose in poseList:
           print " %s" % pose

def save(nav, poseName):
    nav.savePose(poseName)

def delete(nav, poseName):
    if poseName not in nav.poses:
        print "No such pose '%s'" % poseName
    else:
        nav.deletePose(poseName)

def goto(nav, poseName):
    if poseName not in nav.poses:
        print "No such pose '%s'" % poseName
    else:
        nav.goto(poseName)


def help():
    print "Commands:"
    print " list: List saved poses."
    print " save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists."
    print " delete <name>: Delete the pose given by <name>."
    print " goto <name>: Sends the robot to the pose given by <name>."
    print " help: Show this list of commands"
    print " quit: Exits the program"



if __name__ == "__main__":
    nav = NavigationManager()
    nav.loadPoses()
    print "Welcome to the map annotator!"
    help()
    running = True
    while running:
        user_input = raw_input("")
        split = user_input.split(' ', 1)
        command = split[0]
        if command == "list":
            list(nav)
        elif command == "save":
            save(nav, split[1])
        elif command == "delete":
            delete(nav, split[1])
        elif command == "goto":
            goto(nav, split[1])
        elif command == "help":
            help()
        elif command == "quit":
            running = False
        else:
            print "Command not recognized. Type help for options!"



