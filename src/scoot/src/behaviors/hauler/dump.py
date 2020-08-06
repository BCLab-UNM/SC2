#! /usr/bin/env python
"""Dump node."""

from __future__ import print_function
import sys
import rospy
from Scoot import Scoot


def main(task=None):
    if task:
        scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("hauler")
        scoot.start(node_name='dump')
    rospy.loginfo('Dump Started')
    sys.exit(0)  # "succeeded"


if __name__ == '__main__':
    rospy.init_node('dump')
    sys.exit(main())
