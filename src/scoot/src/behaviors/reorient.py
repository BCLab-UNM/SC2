#! /usr/bin/env python3
"""Reorient node."""
 
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
        scoot.start(node_name='reorient')
    rospy.loginfo('Reorientation Started')
    scoot._look(0, math.pi - 0.01)
    scoot._look(0, -math.pi + 0.01)
    scoot.look_forward()
    sys.exit(0)  # "succeeded"

if __name__ == '__main__':
    rospy.init_node('reorient')
    sys.exit(main())
