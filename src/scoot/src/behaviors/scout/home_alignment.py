#! /usr/bin/env python
"""Home Alignment node."""

from __future__ import print_function
import sys
import math
import rospy
from obstacle.msg import Obstacles
from scoot.msg import MoveResult
from Scoot import Scoot, VolatileException, ObstacleException, PathException, AbortException, MoveResult

# @TODO Whiteboard test the angles are not quite right 
def side_align(distance, side, sides, start_pose, home_pose):
    global scoot
    
    scoot.drive(distance, ignore=Obstacles.IS_VOLATILE)
    scoot.set_heading(start_pose.theta + math.pi/2, ignore=-1)

# @TODO Whiteboard test the angles are not quite right 
def main(task=None):
    global scoot
    if task:
        if type(task) == Scoot:
            scoot = task
        else:
            scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("scout_1")
        scoot.start(node_name='home_alignment')
    rospy.loginfo('Home Alignment Started')

    start_pose = scoot.getOdomLocation().getPose()

    #side_len = scoot.getOdomLocation().distance(scoot.home_pose.x, scoot.home_pose.y)
    side_len = 2
    # @TODO: check if need to get closer, also what is a reasonable range
    # @TODO: Set heading torwards home or use scoot.drive_to(scoot.home_pose, distance) where distance is desired
    distance = side_len # @TODO test if this is a reasonable thing to do
    sides = 6
    scoot.set_heading(start_pose.theta + (math.pi / 2.0), ignore=Obstacles.IS_LIDAR) # @TODO: ignore= home | vision | lidar
    scoot.drive(distance/2.0)
    for i in range(1, sides):
        scoot.set_heading(start_pose.theta + (i * (2.0 * math.pi / sides)), ignore=Obstacles.IS_LIDAR) # @TODO: ignore= home | vision | lidar
        scoot.drive(distance)
        # @TODO: Set heading torwards home
        

    
    start_pose = scoot.getOdomLocation().getPose()
    
    sides = 6
    for i in range(0,(sides+1)):
        if i == 0:
            side_align(distance / 2.0, i,sides, start_pose, home_pose)
        else: 
    

    start_pose = scoot.getOdomLocation().getPose()

    #side_len = scoot.getOdomLocation().distance(scoot.home_pose.x, scoot.home_pose.y)
    side_len = 2
    # @TODO: check if need to get closer, also what is a reasonable range
    # @TODO: Set heading torwards home or use scoot.drive_to(scoot.home_pose, distance) where distance is desired
    distance = side_len # @TODO test if this is a reasonable thing to do
    sides = 6
    scoot.set_heading(start_pose.theta + (math.pi / 2.0), ignore=Obstacles.IS_LIDAR) # @TODO: ignore= home | vision | lidar
    scoot.drive(distance/2.0)
    for i in range(1, sides):
        scoot.set_heading(start_pose.theta + (i * (2.0 * math.pi / sides)), ignore=Obstacles.IS_LIDAR) # @TODO: ignore= home | vision | lidar
        scoot.drive(distance)
        # @TODO: Set heading torwards home
        

    sys.exit(0)  # "succeeded"


if __name__ == '__main__':
    rospy.init_node('home_alignment')
    sys.exit(main())
