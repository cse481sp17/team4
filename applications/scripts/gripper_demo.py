#! /usr/bin/env python

import fetch_api
import rospy


def print_usage():
    print 'Usage: rosrun applications gripper_demo.py open'
    print '       rosrun applications gripper_demo.py close 40'


def main():
    rospy.init_node('gripper_demo')
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    command = argv[1]

    gripper = fetch_api.Gripper()
    effort = gripper.MAX_EFFORT
    if command == 'open' and len(argv) > 2:
        effort = float(argv[2])

    if command == 'open':
        rospy.logerr('Not implemented.')
    elif command == 'close':
        rospy.logerr('Not implemented.')
    else:
        print_usage()


if __name__ == '__main__':
    main()
