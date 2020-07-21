#! /usr/bin/env python
"""Fine Search node."""

from __future__ import print_function
import sys
import rospy
from Scoot import Scoot


def main(task=None):
    if task:
        scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("scout_1")
        scoot.start(node_name='fine_search')
    rospy.loginfo('Fine Search Started')
    sys.exit(0)  # "succeeded"


if __name__ == '__main__':
    rospy.init_node('fine_search')
    sys.exit(main())
