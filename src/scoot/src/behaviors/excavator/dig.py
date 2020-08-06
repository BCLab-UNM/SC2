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
    ''' @TODO: Outline 
    Check Bucket status
    Reset to "home"
    Open the Bucket
    Lower the Boom
    Start to Dig - curl the bucket inward, lowering it into the ground to scoop up volatile
    Lift the Load - retract and raise the boom while keeping the bucket curled
    '''
    sys.exit(0)  # "succeeded"


if __name__ == '__main__':
    rospy.init_node('dig')
    sys.exit(main())
