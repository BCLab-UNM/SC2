#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, LaserScan, JointState
from srcp2_msgs.msg import VolSensorMsg

class SensorMonitor:

    def imuCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "IMU data: %s", data)

    def laserScanCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Laser Scan data: %s", data)

    def volatileSensorCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Volatile Sensor data: %s", data)

    def jointStatesCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Joint States data: %s", data)

    def init(self):

        rospy.init_node('sensor_monitor', anonymous=True)

        print("Created Sensor Monitor")

        name = 'scout_1'

        rospy.Subscriber("/{}/imu".format(name), Imu, self.imuCallback)
        rospy.Subscriber("/{}/laser/scan".format(name), LaserScan, self.laserScanCallback)
        rospy.Subscriber("/{}/volatile_sensor".format(name), VolSensorMsg, self.volatileSensorCallback)
        rospy.Subscriber("/{}/joint_states".format(name), JointState, self.jointStatesCallback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

def shutdownHandler():
  print("Sensor monitor shutting down.")

if __name__ == '__main__':

    # Register shutdown handler
    rospy.on_shutdown( shutdownHandler )

    # Initialise the node
    monitor = SensorMonitor()
    monitor.init()
    
