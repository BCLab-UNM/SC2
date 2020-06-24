#!/usr/bin/env python

import rospy, sys, tf
import numpy as np
import angles
from Scoot import VolatileException, ObstacleException, PathException, AbortException, MoveResult
from Scoot import *
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from enum import Enum

current_odom = None

class Span(Enum):
    WALK = 1
    RANGE = 2

def createWaypoint(x, y, altitude):
    return Point(x, y, altitude)

def calculateRange(type, x1, y1, x2, y2, length):
    print "TYPE: {} {} {}".format(type, Span.WALK, Span.RANGE)
    if type == Span.WALK:
        waypoints = []
        print "Calculating walk"
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

            print "{}, {} -> {}, {}".format(previousxoffset, previousyoffset, xoffset, yoffset)

            for (x, y) in calculateRange(rangeType, previousxoffset, previousyoffset, xoffset, yoffset, steplength):
                print "y {}. {}".format(x, y)
                waypoints.append(buildWaypoint(centerx, centery, x * radius, y * radius, altitude))

            previousxoffset = xoffset
            previousyoffset = yoffset

    return waypoints

def position(odom):
    global current_odom
    current_odom = odom

def get2DPose(odometry):
    """gets this robot's position"""
    position = odometry.pose.pose.position
    rotation = odometry.pose.pose.orientation
    # Convert to usable angle
    euler = tf.transformations.euler_from_quaternion(quaternion=(rotation.x, rotation.y, rotation.z, rotation.w))
    theta = euler[2]
    if theta < -math.pi:
        theta = theta + (2*math.pi)
    elif theta > math.pi:
        theta = theta - (2*math.pi)
    return { 'x': position.x, 'y': position.y, 'theta': theta }

def toRadians(degree):
    return 2 * math.pi * degree / 360.0

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def gotowaypoint(waypoint, scoot):
    distance = math.hypot(current_odom.pose.pose.position.x - waypoint.x, current_odom.pose.pose.position.y - waypoint.y)
    while distance > 0.9 :
        pose = get2DPose(current_odom)
        poseAngle = pose['theta']
        # TODO: Remove this when the IMU flip is fixed
        poseAngle = -poseAngle
        angleto = math.atan2(current_odom.pose.pose.position.y - waypoint.y, -current_odom.pose.pose.position.x + waypoint.x)
        angle = angles.shortest_angular_distance(poseAngle, angleto)

        if abs(angle) < toRadians(5):
            # Move forward
            scoot.drive(1)
        elif abs(angle) < toRadians(70):
            # Soft Turn
            scoot.turn(toRadians(-10 if angle > 0 else 10))
        else:
            # Hard Turn
            scoot.turn(toRadians(-50 if angle > 0 else 50))

        distance = math.hypot(waypoint.x - current_odom.pose.pose.position.x, waypoint.y - current_odom.pose.pose.position.y)
    print "Reached waypoint: {}".format(waypoint)




def followDDSA(name):

    global scoot

    ddsaWaypoints = buildDDSAWaypoints(Span.RANGE, 0, 0, 0, 1, 0, 5, 1, 1)

    rospy.Subscriber("{}/odom/filtered".format(name), Odometry, position)
    rospy.wait_for_message("{}/odom/filtered".format(name), Odometry)

    zero_odom = current_odom

    anglediff = angle_between((1, -1, 0), (1, 1, 0))
    print (360 * anglediff / (math.pi * 2))

    for waypoint in ddsaWaypoints :
        print "Going to {}".format(waypoint)
        gotowaypoint(Point(waypoint.x + zero_odom.pose.pose.position.x, waypoint.y + zero_odom.pose.pose.position.y, 0), scoot)



if __name__ == '__main__':
    global scoot
    rospy.init_node('test')
    scoot = Scoot("scout_1")
    scoot.start(node_name='test')

    followDDSA('scout_1')
