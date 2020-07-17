#!/usr/bin/env python
import roslaunch
import rospy
from Driver import State

    
if __name__ == '__main__':
    try:
        rospy.loginfo("MinCore Starting")
        rospy.init_node('MinCore')
        r = rospy.Rate(10) # 10hz
        driver = State()
        rospy.loginfo("MinCore Started")
        task_node = roslaunch.core.Node('scoot', 'Task.py', name='Task', namespace=rospy.get_namespace(), respawn=True)
        launcher = roslaunch.scriptapi.ROSLaunch()
        launcher.start()
        launcher.launch(task_node)
        while not rospy.is_shutdown():
            driver.run()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
