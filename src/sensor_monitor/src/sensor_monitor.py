#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, LaserScan, JointState
from srcp2_msgs.msg import VolSensorMsg

class SensorMonitor:

    def __init__(self, name):
        rospy.Subscriber("/{}/imu".format(name), Imu, self.imuCallback)
        rospy.Subscriber("/{}/laser/scan".format(name), LaserScan, self.laserScanCallback)
        rospy.Subscriber("/{}/volatile_sensor".format(name), VolSensorMsg, self.volatileSensorCallback)
        rospy.Subscriber("/{}/joint_states".format(name), JointState, self.jointStatesCallback)

    def imuCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " IMU data: %s\n", data)

    def laserScanCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Laser Scan data: %s\n", data)

    def volatileSensorCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Volatile Sensor data: %s\n", data)

    def jointStatesCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Joint States data: %s\n", data)


def shutdownHandler():
    print("Sensor monitor shutting down.")

if __name__ == '__main__':

    # Comment added just to work with travis
    
    rospy.init_node('sensor_monitor', anonymous=True)
    print("Created Sensor Monitor")

    # Register shutdown handler
    rospy.on_shutdown( shutdownHandler )

    # Initialise the node
    monitor = SensorMonitor('small_scout_1')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
