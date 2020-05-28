import rospy
from Driver import State
rospy.init_node('corez')
r = rospy.Rate(10) # 10hz
driver = State()

while not rospy.is_shutdown():
    driver.run()
    r.sleep()
