#! /usr/bin/env python3
"""Goto Processing Plant node."""

from __future__ import print_function
import sys
import rospy
from Scoot import Scoot


def main(task=None):
    if task:
        scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("hauler")
        scoot.start(node_name='goto_processing_plant')
    rospy.loginfo('Goto Processing Plant Started')
    sys.exit(0)  # "succeeded"


if __name__ == '__main__':
    rospy.init_node('goto_processing_plant')
    sys.exit(main())
