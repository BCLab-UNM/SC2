#! /usr/bin/env python
"""Dig node."""

from __future__ import print_function
import sys
import rospy
from Scoot import Scoot


def main(task=None):
    if task:
        scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("excavator")
        scoot.start(node_name='dig')
    rospy.loginfo('Dig Started')
    sys.exit(0)  # "succeeded"


if __name__ == '__main__':
    rospy.init_node('dig')
    sys.exit(main())
