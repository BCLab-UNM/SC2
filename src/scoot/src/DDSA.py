#!/usr/bin/env python
import rospy, sys, tf
from Scoot import *
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from enum import Enum
from WaypointNavigator import WaypointNavigator

class Span(Enum):
    WALK = 1
    RANGE = 2

def createWaypoint(x, y, altitude):
    return Point(x, y, altitude)

def calculateRange(type, x1, y1, x2, y2, length):
    # Build the range with either a walk (steps between point 1 and 2) or range (go directly to point 2)
    if type == Span.WALK:
        waypoints = []
        xdistance = x2 - x1
        ydistance = y2 - y1
        distance = math.sqrt((xdistance * xdistance) + (ydistance * ydistance))
        xratio = xdistance / distance
        yratio = ydistance / distance
        for i in range(1, int(distance / length) + 1):
            waypoints.append((x1 + (i * length * xratio),y1 + (i * length * yratio)))
        return waypoints
    elif type == Span.RANGE:
        return [(x2, y2)]

def buildWaypoint(centerx, centery, xoffset, yoffset, altitude):
    return createWaypoint(centerx + xoffset, centery + yoffset, altitude)

def buildDDSAWaypoints(rangeType, centerx, centery, altitude, size, index, loops, radius, steplength):

    waypoints = []
    start = Point(centerx, centery, altitude)
    waypoints.append(start)
    previousxoffset = 0
    previousyoffset = 0
    for loop in range(0, loops):
        for corner in range(0, 4):

            if (loop == 0 and corner == 0):
                xoffset = 0
                yoffset = index + 1
            else:
                xoffset = 1 + index + (loop * size)
                yoffset = xoffset
                if (corner == 0):
                    xoffset = -(1 + index + ((loop - 1) * size))
                elif (corner == 3):
                    xoffset = -xoffset
                if (corner == 2 or corner == 3):
                    yoffset = -yoffset

            for (x, y) in calculateRange(rangeType, previousxoffset, previousyoffset, xoffset, yoffset, steplength):
                waypoints.append(buildWaypoint(centerx, centery, x * radius, y * radius, altitude))

            previousxoffset = xoffset
            previousyoffset = yoffset

    return waypoints

if __name__ == '__main__':
    rospy.init_node('ddsa_navigator')

    rospy.loginfo("DDSA Navigator started.")

    name = 'scout_1'

    scoot = Scoot(name)
    scoot.start(node_name='test')

    navigator = WaypointNavigator(scoot)

    ddsaWaypoints = buildDDSAWaypoints(Span.RANGE,
                                       scoot.OdomLocation.Odometry.pose.pose.position.x,
                                       scoot.OdomLocation.Odometry.pose.pose.position.y,
                                       0, 1, 0, 5, 1, 1)

    rospy.loginfo("Waypoints generated, size: {}".format(len(ddsaWaypoints)))

    rospy.loginfo("Navigating Waypoints...")
    navigator.navigate(ddsaWaypoints)
