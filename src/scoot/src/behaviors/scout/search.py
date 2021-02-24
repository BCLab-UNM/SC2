#! /usr/bin/env python3
"""Search node."""

from __future__ import print_function
import sys
import rospy
from Scoot import Scoot


def main(task=None):
    if task:
        scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("scout_1")
        scoot.start(node_name='Search')
    rospy.loginfo('Search Started')
    sys.exit(0)  # "succeeded"


if __name__ == '__main__':
    rospy.init_node('search')
    sys.exit(main())
