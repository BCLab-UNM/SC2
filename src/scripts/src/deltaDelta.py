#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Odometry

wheel = None
fake  = None
ekf  = None

def dist(p1, p2):
    return math.hypot(p1.x - p2.x, p1.y - p2.y)

def _wheel(msg):
    global wheel
    wheel = msg.pose.pose.position
    
def _fake(msg):
    global fake
    fake = msg.pose.pose.position
    
def _ekf(msg):
    global ekf
    ekf = msg.pose.pose.position
    
def run():
    global wheel
    global fake
    global ekf
    rospy.Subscriber('/scout_1/odom/', Odometry, _wheel)
    rospy.Subscriber('/scout_1/odom/filtered', Odometry, _fake)
    rospy.Subscriber('/scout_1/odometry/filtered', Odometry, _ekf)
    rospy.init_node("odom_dif_test")
    rospy.sleep(0.5)
    delta_f_e = dist(fake, ekf)
    delta_w_e = dist(wheel, ekf)
    delta_f_w = dist(fake, wheel)  
    while not rospy.is_shutdown():
        rospy.sleep(0.3)
        print "delta_f_e:", delta_f_e - dist(fake, ekf)
        print "delta_w_e:", delta_w_e - dist(wheel, ekf)
        print "delta_f_w:", delta_f_w - dist(fake, wheel)

if __name__ == '__main__':
    run()
