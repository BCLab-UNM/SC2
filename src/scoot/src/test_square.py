#! /usr/bin/env python

from __future__ import print_function

import argparse
import sys 
import math 
import rospy 

from obstacle.msg import Obstacles
from geometry_msgs.msg import Pose2D, Point
from Scoot import *

def dumb_square(distance):
    global scoot
    scoot.drive(distance, ignore=Obstacles.IS_VOLATILE)   
    scoot.turn(math.pi/2, ignore=Obstacles.IS_VOLATILE)   

    scoot.drive(distance, ignore=Obstacles.IS_VOLATILE)   
    scoot.turn(math.pi/2, ignore=Obstacles.IS_VOLATILE)   

    scoot.drive(distance, ignore=Obstacles.IS_VOLATILE)   
    scoot.turn(math.pi/2, ignore=Obstacles.IS_VOLATILE)   

    scoot.drive(distance, ignore=Obstacles.IS_VOLATILE)   
    scoot.turn(math.pi/2, ignore=Obstacles.IS_VOLATILE)   

def smart_square(distance, ignore_lidar=False):
    global scoot
    ignore = Obstacles.IS_VOLATILE
    if ignore_lidar:
        ignore |= Obstacles.IS_LIDAR

    # Compute a square based on the current heading. 
    start_pose = scoot.getOdomLocation().getPose()
    start = Point()
    start.x = start_pose.x 
    start.y = start_pose.y
    print('Start point: ({:.2f}, {:.2f})'.format(start.x, start.y))
    
    sq1 = Point()
    sq1.x = start.x + distance * math.cos(start_pose.theta)
    sq1.y = start.y + distance * math.sin(start_pose.theta)
    
    sq2 = Point()
    sq2.x = sq1.x + distance * math.cos(start_pose.theta + math.pi/2)
    sq2.y = sq1.y + distance * math.sin(start_pose.theta + math.pi/2)
    
    sq3 = Point()
    sq3.x = sq2.x + distance * math.cos(start_pose.theta + math.pi)
    sq3.y = sq2.y + distance * math.sin(start_pose.theta + math.pi)

    scoot.drive_to(sq1, ignore=ignore)
    scoot.drive_to(sq2, ignore=ignore)
    scoot.drive_to(sq3, ignore=ignore)
    scoot.drive_to(start, ignore=ignore)

    scoot.set_heading(start_pose.theta)

    end_pose = scoot.getOdomLocation().getPose()
    print('Start point: ({:.2f}, {:.2f})'.format(end_pose.x, end_pose.y))

def abs_square(distance):
    global scoot
    start_pose = scoot.getOdomLocation().getPose()
    
    scoot.drive(distance, ignore=Obstacles.IS_VOLATILE)
    scoot.set_heading(start_pose.theta + math.pi/2, ignore=-1)

    scoot.drive(distance, ignore=Obstacles.IS_VOLATILE)
    scoot.set_heading(start_pose.theta + math.pi, ignore=-1)

    scoot.drive(distance, ignore=Obstacles.IS_VOLATILE)
    scoot.set_heading(start_pose.theta + (3 * math.pi)/2, ignore=-1)

    scoot.drive(distance, ignore=Obstacles.IS_VOLATILE)
    scoot.set_heading(start_pose.theta, ignore=-1)

def main():
    global scoot
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        'distance',
        type=float,
        help='The length (m) of each side of the square.'
    )
    parser.add_argument(
        '--ignore-lidar',
        action='store_true',
        help='Whether the rover should ignore lidar obstacles on its journey.'
    )
    args = parser.parse_args()

    smart_square(args.distance, args.ignore_lidar)

if __name__ == '__main__' :
    global scoot
    rospy.init_node('TestSquareNode')
    scoot = Scoot("scout_1")
    scoot.start()
    main()
