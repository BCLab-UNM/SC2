#!/usr/bin/env python3
from enum import Enum
import math
import rospy
import sys
from Scoot import *
from WaypointNavigator import WaypointNavigator
from geometry_msgs.msg import Point


class Span(Enum):
    WALK = 1
    RANGE = 2


def create_waypoint(x, y, altitude):
    return Point(x, y, altitude)


def calculate_range(range_type, start, end, length):
    # Build the range with either a walk (steps between point 1 and 2) or range (go directly to point 2)
    if range_type == Span.WALK:
        waypoints = []
        print("Calculating walk")
        deltax = end.x - start.x
        deltay = end.y - start.y
        deltaz = end.z - start.z
        distance = math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))
        for i in range(1, int(distance / length) + 1):
            waypoints.append(Point(start.x + (i * length * deltax / distance),
                                   start.y + (i * length * deltay / distance),
                                   start.z + (i * length * deltaz / distance)))
        return waypoints
    elif range_type == Span.RANGE:
        return [end]


def build_waypoint(centerx, centery, xoffset, yoffset, altitude):
    return create_waypoint(centerx + xoffset, centery + yoffset, altitude)


def build_DDSA_waypoints(range_type, loops, altitude=0, size=1, index=0, radius=1, step_length=1):
    waypoints = []
    start = Point(0, 0, altitude)
    waypoints.append(start)
    previous = start
    for loop in range(0, loops):
        for corner in range(0, 4):

            if loop == 0 and corner == 0:
                next = Point(0, index + 1, altitude)
            else:
                xoffset = 1 + index + (loop * size)
                yoffset = xoffset
                if corner == 0:
                    xoffset = -(1 + index + ((loop - 1) * size))
                elif corner == 3:
                    xoffset = -xoffset
                if corner == 2 or corner == 3:
                    yoffset = -yoffset

                next = Point(xoffset, yoffset, altitude)

            for waypoint in calculate_range(range_type, previous, next, step_length):
                waypoints.append(Point(waypoint.x * radius, waypoint.y * radius, waypoint.z))

            previous = next

    return waypoints


# We define main because task expects main()
def main(task=None):
    # Determine if main was called from the shell or by the task object
    if task:
        scoot = task.scoot
    else:  # Called from shell
        # Give the node a name
        name = 'small_scout_1'

        scoot = Scoot(name)
        scoot.start(node_name='test')

    navigator = WaypointNavigator(scoot)

    while scoot.OdomLocation.Odometry is None:
        print("Waiting for initial odom...")
        rospy.sleep(1)

    ddsa_waypoints = build_DDSA_waypoints(Span.RANGE, loops=5)

    ddsa_waypoints = [Point(waypoint.x + scoot.OdomLocation.Odometry.pose.pose.position.x,
                            waypoint.y + scoot.OdomLocation.Odometry.pose.pose.position.y,
                            waypoint.z) for waypoint in ddsa_waypoints]

    rospy.loginfo("Waypoints generated, size: {}".format(len(ddsa_waypoints)))

    rospy.loginfo("Navigating Waypoints...")
    navigator.navigate(ddsa_waypoints)

    # Our exit code convention is that 0 means a volatile was found
    # None 0 means nothing was found
    sys.exit(0)


# Called from the shell
if __name__ == '__main__':
    # Create node for DDSA
    rospy.init_node('ddsa_navigator')
    rospy.loginfo("DDSA Navigator started.")

    # Call main and make its exit code the return value of main
    main()
    sys.exit(0)
