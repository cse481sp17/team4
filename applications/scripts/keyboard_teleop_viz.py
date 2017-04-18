#!/usr/bin/env python

import fetch_api
import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry



import sys, select, termios, tty

msg = """
Control Your Fetch!
---------------------------
Moving around:
        w
   a    s    d

Space: force stop
i/k: increase/decrease only linear speed by 5 cm/s
u/j: increase/decrease only angular speed by 0.25 rads/s
anything else: stop smoothly

CTRL-C to quit
"""

moveBindings = {'w': (1, 0), 'a': (0, 1), 'd': (0, -1), 's': (-1, 0)}

speedBindings = {
    'i': (0.05, 0),
    'k': (-0.05, 0),
    'u': (0, 0.25),
    'j': (0, -0.25),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = .5
turn = 1
all_points = [Point(0.0, 0.0, 0.0)]


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def show_text_in_rviz(text):
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=text)
    marker_publisher.publish(marker)
    
    
def show_path_in_rviz(data):
	# Get the last point and the new point
    last_point = all_points[len(all_points) - 1]
    next_point = data.pose.pose.position
    change = (last_point.x - next_point.x) ** 2 + (last_point.y - next_point.y) ** 2 + (last_point.z - next_point.z) ** 2
    # Only add the point if is is a large enough change
    if change > 0.1:
    	all_points.append(next_point)
    default_pose = Pose()
    # create and publish the marker
    marker = Marker(
	type=Marker.LINE_STRIP,
	id=1,
	lifetime=rospy.Duration(0),
	pose=default_pose,
	scale=Vector3(0.3, 0.3, 0.3),
	header=Header(frame_id=data.header.frame_id),
	color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
	points=all_points)
    marker_publisher.publish(marker)



if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('fetch_teleop_key')
    base = fetch_api.Base()
    rospy.Subscriber('/odom', Odometry, show_path_in_rviz)
    
    x = 0
    th = 0
    status = 0
    count = 0
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print msg
        print vels(speed, turn)
        marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        while (1):
            show_text_in_rviz("test")
        	# print "enter msg for rviz text"
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed += speedBindings[key][0]
                turn += speedBindings[key][1]
                count = 0

                print vels(speed, turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key == ' ':
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + 0.02)
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - 0.02)
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min(target_turn, control_turn + 0.1)
            elif target_turn < control_turn:
                control_turn = max(target_turn, control_turn - 0.1)
            else:
                control_turn = target_turn
            base.move(control_speed, control_turn)
    except e:
        rospy.logerr('{}'.format(e))
    finally:
        base.stop()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
