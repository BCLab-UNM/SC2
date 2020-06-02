#!/usr/bin/env python
import rospy
from Driver import State

    
if __name__ == '__main__':
    try:
        rospy.loginfo("MinCore Starting")
        rospy.init_node('MinCore')
        r = rospy.Rate(10) # 10hz
        driver = State()
        rospy.loginfo("MinCore Started")
        while not rospy.is_shutdown():
            driver.run()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
