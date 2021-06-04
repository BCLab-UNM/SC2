#! /usr/bin/env python3
"""Repair node."""
 
from __future__ import print_function
import sys
import rospy
from Scoot import Scoot
import math

def main(task=None):
    if task:
        scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot(rospy.get_namespace().strip('/'))
        scoot.start(node_name='repair')
    rospy.loginfo('Repair Started')
    sys.exit(0)  # "succeeded"

if __name__ == '__main__':
    rospy.init_node('repair')
    sys.exit(main())
