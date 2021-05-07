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
    rospy.loginfo('Goto Repair Station Started')
    repair_station_pose = scoot.repair_station_pose
    go_to.main()
    result = go_to.goto(repair_station_pose.x, repair_station_pose.y, 0, 0)

    scoot.drive(0, ignore=Obstacles.IS_LIDAR | Obstacles.IS_VOLATILE)
    scoot.brake()
       
    if result:
        rospy.loginfo('goto_repair_station: succeeded')
        sys.exit(0)
    else:
        rospy.loginfo('goto_repair_station: failed')
        sys.exit(1)


if __name__ == '__main__':
    rospy.init_node('goto_repair_station')
    sys.exit(main())
