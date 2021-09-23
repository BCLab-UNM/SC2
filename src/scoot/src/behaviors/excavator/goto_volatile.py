#! /usr/bin/env python3
"""Goto Volatile node."""

import sys
import rospy
import math
from Scoot import Scoot, Location, VolatileException, ObstacleException

def main(task=None):
    if task:
        if type(task) == Scoot:
            scoot = task
        else:
            scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("excavator")
        scoot.start(node_name='goto_volatile')
    rospy.loginfo('Goto Volatile Started')
    scoot.move_shoulder_yaw(-math.pi)
    rospy.sleep(3)
    vol_pose = scoot.get_closest_vol_point()
    # @TODO make a while loop or change state meh
    if vol_pose:
        try:
            scoot.drive_to(vol_pose)
        except ObstacleException:
            pass
        except VolatileException:
            sys.exit(0)  # "succeeded" # @TODO: might retest mass as this volatile might be almost exhausted
        # @TODO: obstacle avoidance calls should live here
        if scoot.OdomLocation.at_goal(vol_pose, 2.0):
            sys.exit(0)  # "succeeded"
    else: # no volitile
        rospy.sleep(5)
    sys.exit(-1)  # "failed"


if __name__ == '__main__':
    rospy.init_node('goto_volatile')
    sys.exit(main())
