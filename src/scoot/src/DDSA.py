#!/usr/bin/env python
import rospy, sys, tf
from Scoot import *
from geometry_msgs.msg import Point
from enum import Enum
from WaypointNavigator import WaypointNavigator

class Span(Enum):
    WALK = 1
    RANGE = 2

def createWaypoint(x, y, altitude):
    return Point(x, y, altitude)

def calculateRange(type, start, end, length):
    # Build the range with either a walk (steps between point 1 and 2) or range (go directly to point 2)
    if type == Span.WALK:
        waypoints = []
        print "Calculating walk"
        deltax = end.x - start.x
        deltay = end.y - start.y
        deltaz = end.z - start.z
        distance = math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))
        for i in range(1, int(distance / length) + 1):
            waypoints.append(Point(start.x + (i * length * deltax / distance),
                                   start.y + (i * length * deltay / distance),
                                   start.z + (i * length * deltaz / distance)))
        return waypoints
    elif type == Span.RANGE:
        return [end]

def buildWaypoint(centerx, centery, xoffset, yoffset, altitude):
    return createWaypoint(centerx + xoffset, centery + yoffset, altitude)

def buildDDSAWaypoints(rangeType, loops, altitude=0, size=1, index=0, radius=1, steplength=1):

    waypoints = []
    start = Point(0, 0, altitude)
    waypoints.append(start)
    previous = start
    for loop in range(0, loops):
        for corner in range(0, 4):

            if (loop == 0 and corner == 0):
                next = Point(0, index + 1, altitude)
            else:
                xoffset = 1 + index + (loop * size)
                yoffset = xoffset
                if (corner == 0):
                    xoffset = -(1 + index + ((loop - 1) * size))
                elif (corner == 3):
                    xoffset = -xoffset
                if (corner == 2 or corner == 3):
                    yoffset = -yoffset

                next = Point(xoffset, yoffset, altitude)

            for waypoint in calculateRange(rangeType, previous, next, steplength):
                waypoints.append(Point(waypoint.x * radius, waypoint.y * radius, waypoint.z))

            previous = next

    return waypoints

if __name__ == '__main__':
    rospy.init_node('ddsa_navigator')

    rospy.loginfo("DDSA Navigator started.")

    name = 'scout_1'

    scoot = Scoot(name)
    scoot.start(node_name='test')

    navigator = WaypointNavigator(scoot)

    while scoot.OdomLocation.Odometry == None:
        print "Waiting for inital odom..."
        rospy.sleep(1)

    ddsaWaypoints = buildDDSAWaypoints(Span.RANGE, loops=5)

    ddsaWaypoints = [Point(waypoint.x + scoot.OdomLocation.Odometry.pose.pose.position.x,
                           waypoint.y + scoot.OdomLocation.Odometry.pose.pose.position.y,
                           waypoint.z) for waypoint in ddsaWaypoints]

    rospy.loginfo("Waypoints generated, size: {}".format(len(ddsaWaypoints)))

    rospy.loginfo("Navigating Waypoints...")
    navigator.navigate(ddsaWaypoints)
