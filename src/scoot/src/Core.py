#!/usr/bin/env python3
import roslaunch
import rospy
from Driver import State

    
if __name__ == '__main__':
    try:
        rospy.loginfo("Core Starting")
        rospy.init_node('Core')
        r = rospy.Rate(10)  # 10hz
        driver = State()
        rospy.loginfo("Core Started")
        if rospy.get_param("mode", default="auto").upper() == "AUTO":
            rospy.loginfo("Core Starting with Task")
            task_node = roslaunch.core.Node('scoot', 'Task.py', name='Task', namespace=rospy.get_namespace(),
                                            respawn=True)
            launcher = roslaunch.scriptapi.ROSLaunch()
            launcher.start()
            launcher.launch(task_node)
        else:
            rospy.loginfo("Core Starting without Task")

        while not rospy.is_shutdown():
            driver.run()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
