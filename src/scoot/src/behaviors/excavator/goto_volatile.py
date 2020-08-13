#! /usr/bin/env python
"""Goto Volatile node."""

from __future__ import print_function
import sys

import rospy
from Scoot import Scoot, Location, VolatileException, ObstacleException

def main(task=None):
    if task:
        scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("excavator")
        scoot.start(node_name='goto_volatile')
    rospy.loginfo('Goto Volatile Started')
    # test if I need to do .pose.position or something
    vol_pose = scoot.get_closest_vol()
    try:
        scoot.drive_to(vol_pose)
    except ObstacleException:
        pass
    except VolatileException:
        sys.exit(0)  # "succeeded" # @TODO: might retest mass as this volatile might be almost exhausted
    # @TODO: obstacle avoidance calls should live here
    if scoot.OdomLocation.atGoal(vol_pose, 2.0):
        sys.exit(0)  # "succeeded"
    else:
        sys.exit(-1)  # "failed"


if __name__ == '__main__':
    rospy.init_node('goto_volatile')
    sys.exit(main())
