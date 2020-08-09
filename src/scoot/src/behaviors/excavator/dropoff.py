#! /usr/bin/env python
"""Dropoff node."""

from __future__ import print_function
import sys
import rospy
import math
from Scoot import Scoot


def main(task=None):
    if task:
        scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("excavator")
        scoot.start(node_name='dropoff')
    rospy.loginfo('Dropoff Started')
    # @TODO check if have anything in bucket, verify state meh can can handle a value for said state
    # @TODO: check for message from hauler
    rospy.sleep(1)
    scoot.move_mount(math.pi)
    scoot.move_bucket(0)  # dump vol
    # @TODO check scoot.bucket_info().mass_in_bucket < 1 and might check hauler for logging purposes
    sys.exit(0)  # "succeeded"


if __name__ == '__main__':
    rospy.init_node('dropoff')
    sys.exit(main())
