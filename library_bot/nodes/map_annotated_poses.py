#! /usr/bin/env python

import rospy
import geometry_msgs.msg
import move_base_msgs.msg

from web_nav_manager import WebNavigationManager 

import pickle

nav_manager = WebNavigationManager()
nav_manager.loadPoses()

for pose in nav_manager.listPoses():
    print pose
    print nav_manager.poses[pose]
    print ""