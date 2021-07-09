#! /usr/bin/env python3
"""Search node."""

import sys
import rospy
import math
import random

from obstacle.msg import Obstacles
from scoot.msg import MoveResult

from Scoot import VolatileException, ObstacleException, PathException, AbortException, MoveResult, DriveException, Scoot

ignoring = 0


def turnaround(ignore=Obstacles.IS_LIDAR | Obstacles.IS_VOLATILE):
    global scoot
    global ignoring
    scoot.drive(-0.275 * 2, ignore=ignore | ignoring)  # back up by wheel diameter
    scoot.turn(random.gauss(math.pi / 2, math.pi / 4), ignore=ignore | ignoring)
    scoot._look(math.pi / 7, 0)  # look down-ish pi/8 did not see the rocks and pi/6 saw the ground


def wander():
    global scoot
    global ignoring
    try:
        rospy.loginfo("Wandering...")
        scoot.drive(random.gauss(25, 5), ignore=ignoring)
        scoot.turn(random.gauss(-math.pi / 3, math.pi / 3), ignore=ignoring)

    except ObstacleException:
        rospy.loginfo("I saw an obstacle!")
        turnaround()
        rospy.sleep(.5)


def random_walk(num_moves):
    """Do random walk `num_moves` times."""
    global scoot
    global ignoring
    scoot._look(math.pi / 7, 0)  # look down-ish pi/8 did not see the rocks and pi/6 saw the ground
    rospy.sleep(3)  # @TODO look at joint state to se if sleep is needed
    try:
        for move in range(num_moves):
            if rospy.is_shutdown():
                sys.exit(-1)
            wander()
    except VolatileException:
        rospy.logwarn("I found a volatile! " + scoot.VOL_TYPES[scoot.control_data])
        # locking behavior so only one is accessing or writing to the volatile_locations ros param at a time
        try:
            while rospy.get_param("/volatile_locations_latch", default=False):
                rospy.sleep(0.2)  # wait for it be be unlatched
            rospy.set_param('/volatile_locations_latch', True)  # this is to support multiple rovers
            volatile_locations = rospy.get_param("/volatile_locations", default=list())
            volatile_locations.append({'data': scoot.control_data,
                                       'x': scoot.get_odom_location().get_pose().x,
                                       'y': scoot.get_odom_location().get_pose().y})
            rospy.set_param('/volatile_locations', volatile_locations)
        finally:
            rospy.set_param('/volatile_locations_latch', False)

        # move away before exiting
        ignoring |= Obstacles.IS_VOLATILE
        wander()
        rospy.logwarn('Exiting')
        sys.exit(0)


def main(task=None):
    global scoot
    global ignoring
    if task:
        if type(task) == Scoot:
            scoot = task
        else:
            scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("small_scout_1")
        scoot.start(node_name='search')
    rospy.loginfo("Search Node Started")
    # reset ignoring
    ignoring = Obstacles.CUBESAT | Obstacles.HOME_FIDUCIAL | Obstacles.HOME_LEG | Obstacles.VISION_VOLATILE

    random_walk(num_moves=50)
    scoot.brake()
    rospy.loginfo("I'm probably lost!")  # @ TODO add a reorient state in the task state meh
    sys.exit(1)


if __name__ == '__main__':
    global scoot
    rospy.init_node('search')
    sys.exit(main())
