#! /usr/bin/env python
"""Search node."""

from __future__ import print_function
import sys
import rospy
import math
import random 


from obstacle.msg import Obstacles
from scoot.msg import MoveResult

from Scoot import VolatileException, ObstacleException, PathException, AbortException, MoveResult

def turnaround(ignore=Obstacles.IS_LIDAR|Obstacles.IS_VOLATILE):
    global scoot
    scoot.turn(random.gauss(math.pi/2, math.pi/4), ignore=ignore)

def wander():
    global scoot
    try:
        rospy.loginfo("Wandering...")
        scoot.drive(random.gauss(4, 1))
        
    except ObstacleException:
        print ("I saw an obstacle!")
        turnaround()
        rospy.sleep(.5)


def random_walk(num_moves):
    """Do random walk `num_moves` times."""
    global scoot
    try:
        for move in range(num_moves):
            if rospy.is_shutdown():
                sys.exit(-1)
            wander()
    except VolatileException:
        print("I found a volatile!", scoot.VOL_TYPES[scoot.control_data])
        scoot.score(scoot.control_data)
        sys.exit(0)


def main(shared_scoot):
    global scoot
    scoot = shared_scoot
    print ("Search Node Started")
    random_walk(num_moves=50)
    print ("I'm probably lost!")
    sys.exit(1)


if __name__ == '__main__':
    global scoot
    rospy.init_node('search')
    scoot = Scoot("scout_1") 
    scoot.start(node_name='search')
    sys.exit(main())
