#! /usr/bin/env python
"""Search node."""

import sys
import rospy
import math
import random

from obstacle.msg import Obstacles
from scoot.msg import MoveResult

from Scoot import VolatileException, ObstacleException, PathException, AbortException, MoveResult, \
    VisionVolatileException, CubesatException, HomeLegException, HomeLogoException, DriveException, Scoot

ignoring = 0


def turnaround(ignore=Obstacles.IS_LIDAR | Obstacles.IS_VOLATILE):
    global scoot
    global ignoring
    scoot.drive(-0.275 * 2, ignore=ignore | ignoring | Obstacles.CUBESAT)  # back up by wheel diameter

    scoot.lookUp()
    scoot.turn(random.gauss(math.pi / 2, math.pi / 4), ignore=ignore | ignoring)
    scoot.lookForward()


def wander():
    global scoot
    global ignoring
    try:
        # Look up and spin around looking for cubesat if not found
        if scoot.ROUND_NUMBER == 3 and not scoot.cubesat_found:
            rospy.loginfo("Spinning...")
            scoot.lookUp()
            # if this finds a cubesat it will go down to random_walk's except CubesatException handler
            heading = scoot.getOdomLocation().getPose().theta
            scoot.timed_drive(10, 0, scoot.TURN_SPEED, ignore=ignoring)
            scoot.set_heading(heading)  # restore heading
            scoot.lookForward()
        rospy.loginfo("Wandering...")
        # @NOTE: The cubesat has alot of false postives when not looking up
        scoot.drive(random.gauss(4, 1), ignore=ignoring | Obstacles.CUBESAT)

    except ObstacleException:
        rospy.loginfo("I saw an obstacle!")
        turnaround()
        rospy.sleep(.5)


def random_walk(num_moves):
    """Do random walk `num_moves` times."""
    global scoot
    scoot.lookForward()
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
    except CubesatException as e:
        rospy.loginfo("Found CubeSat turning to center")
        scoot.turn(-e.heading, ignore=ignoring | Obstacles.CUBESAT)
        rospy.sleep(1)
        try:
            scoot.timed_drive(3, 0, 0.1, ignore=ignoring)  # so we can update the cubesat point if its still in view
        except DriveException:
            pass
        scoot.brake()
        scoot.score_cubesat()
        scoot.lookForward()
        sys.exit(0)
    except HomeLegException as e:
        rospy.loginfo("Found Home's leg turning to center and going to")
        scoot._light(0.2)
        # turn and drive to it, score
        try:
            scoot.turn(-e.heading, ignore=Obstacles.HOME_LEG | ignoring)
            scoot.drive(e.distance, ignore=Obstacles.HOME_LEG | ignoring)
        except HomeLogoException as e:
            home_exception_behavior(e)
        scoot.score_home_arrive()
        sys.exit(0)
    except HomeLogoException as e:
        home_exception_behavior(e)
        sys.exit(0)


def home_exception_behavior(e):
    global ignoring
    rospy.loginfo("Found Home's Logo turning to center and going to")
    # turn and drive to it, score
    try:
        scoot.turn(-e.heading, ignore=ignoring | Obstacles.HOME_FIDUCIAL | Obstacles.HOME_LEG)
        scoot.drive(e.distance, ignore=ignoring | Obstacles.HOME_FIDUCIAL | Obstacles.HOME_LEG)
    except ObstacleException:
        pass  # This is expected as we are driving until we run into the logo
    finally:
        scoot.brake()
    scoot.score_home_aligned()


def main(task=None):
    global scoot
    global ignoring  # this is based on the round and phase of the round
    if task:
        if type(task) == Scoot:
            scoot = task
        else:
            scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("scout_1")
        scoot.start(node_name='search')
    rospy.loginfo("Search Node Started")
    ignoring = 0
    if scoot.ROUND_NUMBER == 1:
        ignoring |= Obstacles.CUBESAT | Obstacles.HOME_FIDUCIAL | Obstacles.HOME_LEG | Obstacles.VISION_VOLATILE
    elif scoot.ROUND_NUMBER == 3:
        ignoring |= Obstacles.VOLATILE | Obstacles.VISION_VOLATILE
        scoot.light_off()
        if not scoot.cubesat_found:
            ignoring |= Obstacles.HOME_LEG | Obstacles.HOME_FIDUCIAL
        elif not scoot.home_arrived:
            scoot._light(0.2)
            ignoring |= Obstacles.CUBESAT

    random_walk(num_moves=50)
    scoot.brake()
    rospy.loginfo("I'm probably lost!")
    sys.exit(1)


if __name__ == '__main__':
    global scoot
    rospy.init_node('search')
    sys.exit(main())
