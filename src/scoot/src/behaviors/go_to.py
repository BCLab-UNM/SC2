#! /usr/bin/env python
"""Waypoint stub node."""

from __future__ import print_function
import sys
import rospy
import math
from Scoot import Scoot
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import String
from Scoot import Scoot, Location
from nav_msgs.msg import Odometry

scoot = None
pub = None
status_topic = '/scout_1/bug_nav_status'
last_status_msg = None

def goto(x, y, timeout, tolerance):
    global last_status_msg
    global scoot
    wp = Point(x, y, 0)
    while not rospy.is_shutdown():
        pub.publish(wp)

    # Wait for bug_nav to return a status
    rospy.loginfo('waiting for status')
    rospy.wait_for_message( status_topic, String )
    print(last_status_msg)
    if last_status_msg == "Arrived!":
        print('returned true')
        return True
    else:
        return False

def status_handler(msg):
    global last_status_msg
    rospy.logwarn("Received status from bug nav: " + str(msg.data))
    last_status_msg = msg.data

def main(task=None):
    global pub
    global scoot
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
