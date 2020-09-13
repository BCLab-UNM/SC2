#! /usr/bin/env python
"""Search node."""

import sys
import rospy
import math
import random 


from obstacle.msg import Obstacles
from scoot.msg import MoveResult

from Scoot import VolatileException, ObstacleException, PathException, AbortException, MoveResult, \
    VisionVolatileException, CubesatException, HomeLegException, HomeLogoException
ignoring = 0


def turnaround(ignore=Obstacles.IS_LIDAR | Obstacles.IS_VOLATILE):
    global scoot
    global ignoring
    scoot.drive(-0.275 * 2, ignore=ignore | ignoring)  # back up by wheel diameter
    scoot.turn(random.gauss(math.pi/2, math.pi/4), ignore=ignore)


def wander():
    global scoot
    global ignoring
    try:
        rospy.loginfo("Wandering...")
        scoot.drive(random.gauss(4, 1), ignoring)
        
    except ObstacleException:
        rospy.loginfo("I saw an obstacle!")
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
        rospy.logwarn("I found a volatile! " + scoot.VOL_TYPES[scoot.control_data])
        res = scoot.score(scoot.control_data)
        if not res:
            rospy.logwarn("Turning around")
            turnaround()
            rospy.sleep(.5)
        rospy.logwarn('Exiting')
        sys.exit(0)
    except VisionVolatileException as e:
        pass  # @NOTE: not going to use until odom is good
    except CubesatException:
        # @TODO turn, sleep, score & profit
        pass
    except HomeLegException as e:
        # @TODO turn and drive to it
        pass
    except HomeLogoException as e:
        # @TODO turn and drive to it
        pass


def main(task=None):
    global scoot
    global ignoring  # this is based on the round and phase of the round
    if task:
        scoot = task.scoot
    rospy.loginfo("Search Node Started")
    ignoring = 0
    if scoot.ROUND_NUMBER == 1:
        ignoring |= Obstacles.CUBESAT | Obstacles.HOME_FIDUCIAL | Obstacles.HOME_LEG | Obstacles.VISION_VOLATILE
    elif scoot.ROUND_NUMBER == 3:
        ignoring |= Obstacles.VOLATILE | Obstacles.VISION_VOLATILE
        if scoot.cubesat_found:
            ignoring |= Obstacles.CUBESAT
        if scoot.home_found:
            ignoring |= Obstacles.CUBESAT

    random_walk(num_moves=50)
    scoot.brake()
    rospy.loginfo("I'm probably lost!")
    sys.exit(1)


if __name__ == '__main__':
    global scoot
    rospy.init_node('search')
    scoot = Scoot("scout_1") 
    scoot.start(node_name='search')
    sys.exit(main())
