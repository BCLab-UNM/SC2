#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
import math

from obstacle.msg import Obstacles

from Scoot import ObstacleException, Scoot

def bug_algorithm(locations, bug_type):
    global scoot

    if bug_type == 0:
        bug_0(locations)
    elif bug_type == 1:
        bug_1(locations)
                
def bug_0(loc):
    global scoot

    rospy.sleep(0.5)
    while not scoot.getOdomLocation().atGoal(loc, distance=2):
        try:
            rospy.loginfo('driving')
            scoot.drive_to(waypoint)
        except ObstacleException as e:
            if e.obstacle & Obstacles.LIDAR_LEFT == Obstacles.LIDAR_LEFT:
                rospy.loginfo('turn right')
                scoot.turn(-math.pi / 6, ignore=Obstacles.IS_LIDAR)
                try:
                    rospy.loginfo('drive straight')
                    scoot.drive(0.25)
                except ObstacleException:
                    rospy.loginfo('another obstacle')
                    pass
            elif e.obstacle & Obstacles.LIDAR_RIGHT == Obstacles.LIDAR_RIGHT:
                rospy.loginfo('turn left')
                scoot.turn(math.pi / 6, ignore=Obstacles.IS_LIDAR)
                try:
                    rospy.loginfo('drive straight')
                    scoot.drive(0.25)
                except ObstacleException:
                    rospy.loginfo('another obstacle')
                    pass
            elif e.obstacle & Obstacles.LIDAR_CENTER == Obstacles.LIDAR_CENTER:
                rospy.loginfo('turn left')
                scoot.turn(math.pi / 6, ignore=Obstacles.IS_LIDAR)
                try:
                    rospy.loginfo('drive straight')  
                    scoot.drive(0.25)
                except ObstacleException:
                    rospy.loginfo('another obstacle')
                    pass
    rospy.loginfo('Reached waypoint')
    rospy.loginfo("Waypoint: " + str(waypoint.x) + ", " +  str(waypoint.y))
    rospy.loginfo("My loc: " + str(scoot.getOdomLocation().getPose().x) + ", " + str(scoot.getOdomLocation().getPose().y))
    
                
def main(task=None):
    global scoot
    scoot = Scoot("excavator_1")
    scoot.start(node_name='waypoint_search')
    if task:
        scoot = task.scoot
    rospy.loginfo("Waypoint Node Started")
    while not rospy.is_shutdown():
        loc = scoot.get_closest_vol_pose()
        if loc == None:
            rospy.loginfo('Aw :(')
        bug_algorithm(loc, 0)
    rospy.loginfo("I'm probably lost!")
    sys.exit(1)

if __name__ == '__main__':
    global scoot
    rospy.init_node('waypoint_search')
    sys.exit(main())
