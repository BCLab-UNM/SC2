#! /usr/bin/env python
"""Waypoint stub node."""

from __future__ import print_function
import sys
import rospy
import math
from Scoot import Scoot
from geometry_msgs.msg import Point
from std_msgs.msg import String

pub = None
status_topic = '/scout_1/bug_nav_status'
last_status_msg = None

def goto(x, y, timeout, tolerance):
    wp = Point(x, y, 0)
    pub.publish(wp)
    rospy.loginfo('published waypoint')

    # Wait for bug_nav to return a status
    rospy.wait_for_message( status_topic, String )
    if  last_status_msg == "Arrived":
        return True
    else:
        return False

def status_handler(msg):
    rospy.logwarn("Received status from bug nav: " + str(msg))
    last_status_msg = msg

def main(task=None):
    global pub
    pub = rospy.Publisher('/scout_1/waypoints', Point, queue_size=1)
    sub = rospy.Subscriber(status_topic, String, status_handler)

    rospy.logwarn("Publishing status messages on" + status_topic)

    if task:
        if type(task) == Scoot:
            scoot = task
        else:
            scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("scout_1")
        scoot.start(node_name='waypoint_stub')
    rospy.loginfo('Waypoint scoot stub started')
    if rospy.is_shutdown():
        rospy.loginfo('shutdown')
        sys.exit(0)  # "succeeded

if __name__ == '__main__':
    rospy.init_node('waypoint_stub')
    sys.exit(main())
