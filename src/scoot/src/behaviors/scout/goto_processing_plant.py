#! /usr/bin/env python
"""Goto Processing Plant node."""

from __future__ import print_function
import sys
import rospy
from Scoot import Scoot
from .. import go_to


def main(task=None):
    if task:
        if type(task) == Scoot:
            scoot = task
        else:
            scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("scout")
        scoot.start(node_name='goto_processing_plant')
    rospy.loginfo('Goto Processing Plant Started')
    home_pose = scoot.home_pose
    print(scoot.home_pose.x)
    go_to.main()
    print(home_pose.x)
    result = go_to.goto(home_pose.x, home_pose.y, 0, 0)

    if result:
        print('exited')
        sys.exit(0)  # "succeeded"


if __name__ == '__main__':
    rospy.init_node('goto_processing_plant')
    sys.exit(main())
