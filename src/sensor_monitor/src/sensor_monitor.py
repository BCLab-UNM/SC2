#!/usr/bin/env python

# This node 

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def sensorMonitor():

    rospy.init_node('sensor_monitor', anonymous=True)

    print("Created Sensor Monitor")
    
    # rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def shutdownHandler():
  print("Sensor monitor shutting down.")

if __name__ == '__main__':

    # Register shutdown handler
    rospy.on_shutdown( shutdownHandler )

    # Initialise the node
    sensorMonitor()
    
