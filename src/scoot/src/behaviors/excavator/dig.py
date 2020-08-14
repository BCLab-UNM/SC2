#! /usr/bin/env python
"""Dig node."""

from __future__ import print_function
import sys
import rospy
import math
from Scoot import Scoot


def main(task=None):
    sleepy_time = 5
    if task:
        if type(task) == Scoot:
            scoot = task
        else:
            scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("excavator")
        scoot.start(node_name='dig')
    rospy.loginfo('Dig Started')
    scoot.brake()
    # Check Bucket status
    if scoot.bucket_info().mass_in_bucket <= 10:
        # Reset to "home"
        scoot.move_base_arm(0)
        scoot.move_distal_arm(0)
        scoot.move_mount(0)
        # Open the Bucket
        scoot.move_bucket(0)
        rospy.sleep(sleepy_time)
        # Lower the Boom
        scoot.move_base_arm(math.pi/5)  # Make sure to not hit the sensors
        rospy.sleep(sleepy_time)
        # Start to Dig - curl the bucket inward, lowering it into the ground to scoop up volatile
        scoot.move_bucket(math.pi)
        rospy.sleep(sleepy_time)
        # @TODO check bucket and try again or proceed
        # Lift the Load - retract and raise the boom while keeping the bucket curled
        scoot.move_distal_arm(math.pi/4)  # moves arm/bucket inward
        scoot.move_base_arm(0)  # moves arm/bucket up 
        rospy.sleep(sleepy_time)
        # Curl Bucket more and fully extend
        scoot.move_bucket((5 * math.pi)/4)
        scoot.move_distal_arm(0)
    if scoot.bucket_info().mass_in_bucket > 10:  # should be below 50 but the higher the better
        sys.exit(0)  # "succeeded"
        
    sys.exit(1)  # failed


if __name__ == '__main__':
    rospy.init_node('dig')
    sys.exit(main())
