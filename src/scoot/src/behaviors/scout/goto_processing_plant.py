#! /usr/bin/env python3
"""Goto Processing Plant node."""

import sys
import rospy
from Scoot import Scoot
from .. import go_to
from obstacle.msg import Obstacles


def main(task=None):
    if task:
        if type(task) == Scoot:
            scoot = task
        else:
            scoot = task.scoot
    rospy.loginfo('Goto Processing Plant Started')
    home_pose = scoot.home_pose
    go_to.main()
    result = go_to.goto(home_pose.x, home_pose.y, 0, 0)

    scoot.drive(0, ignore=Obstacles.IS_LIDAR|Obstacles.IS_VOLATILE)
    scoot.brake()
       
    if result:
        rospy.loginfo('goto_processing_plant: succeeded')
        sys.exit(0)
    else:
        rospy.loginfo('goto_processing_plant: failed')
        sys.exit(1)


if __name__ == '__main__':
    rospy.init_node('goto_processing_plant')
    sys.exit(main())
