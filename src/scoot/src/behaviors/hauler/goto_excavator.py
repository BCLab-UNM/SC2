#! /usr/bin/env python3
"""Goto Excavator node."""

from __future__ import print_function
import sys
import rospy
from Scoot import Scoot


def main(task=None):
    if task:
        scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("hauler")
        scoot.start(node_name='goto_excavator')
    rospy.loginfo('Goto Excavator Started')
    sys.exit(0)  # "succeeded"


if __name__ == '__main__':
    rospy.init_node('goto_excavator')
    sys.exit(main())