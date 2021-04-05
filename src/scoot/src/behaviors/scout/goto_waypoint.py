#! /usr/bin/env python3
"""Goto Waypoint node."""

import sys
import rospy
from Scoot import Scoot, Location, VolatileException, ObstacleException
from .. import go_to


def main(task=None):
    if task:
        if type(task) == Scoot:
            scoot = task
        else:
            scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("small_scout_1")
        scoot.start(node_name='goto_waypoint')
    rospy.loginfo('Goto Waypoint Started')
    # vol_pose = scoot.get_closest_vol_pose()
    go_to.main()
    result = go_to.goto(20, 20, 0, 0)

    if result:
        sys.exit(0)  # "succeeded" 


if __name__ == '__main__':
    rospy.init_node('goto_waypoint')
    sys.exit(main())
