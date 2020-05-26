#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, LaserScan, JointState
# from srcp2_msgs import VolSensorMsg


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def imuCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

def laserScanCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

def volatileSensorCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

def jointStatesCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

def sensorMonitor():

    rospy.init_node('sensor_monitor', anonymous=True)

    print("Created Sensor Monitor")

    name = 'scout_1'

    rospy.Subscriber("/{}/imu".format(name), Imu, imuCallback)
    rospy.Subscriber("/{}/laser/scan".format(name), LaserScan, laserScanCallback)
    # rospy.Subscriber("/{}/volatile_gisensor".format(name), VolSensorMsg, volatileSensorCallback)
    rospy.Subscriber("/{}/joint_states".format(name), Imu, jointStatesCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def shutdownHandler():
  print("Sensor monitor shutting down.")

if __name__ == '__main__':

    # Register shutdown handler
    rospy.on_shutdown( shutdownHandler )

    # Initialise the node
    sensorMonitor()
    
