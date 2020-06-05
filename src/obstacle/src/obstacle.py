#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from srcp2_msgs.msg import VolSensorMsg
from obstacle.msg import Obstacle
#from obstacle import msg
# @TODO Track down bug with "ImportError: No module named msg"


# @TODO: check/test for syntax errors
# @TODO: check/test for logic errors someone else
# @TODO: test in sim 
# @TODO: replace all magic numbers with ros params in launch file

class Obstacle:

    def __init__(self, name):
        self.rover_width = rospy.get_param('/rover_total_width', default=2.2098)
        self.laserOverlap = 7.0 / 16.0  # @TODO: need a better name
        self.laserscan = LaserScan()
        self.volSensor = VolSensorMsg()
        self.obstacleAccumulator = Obstacle.PATH_IS_CLEAR
        self.obstacleMask = Obstacle.PATH_IS_CLEAR
        rospy.Subscriber("/{}/laser/scan".format(name), LaserScan, self.laserScanCallback)
        rospy.Subscriber("/{}/volatile_sensor".format(name), VolSensorMsg, self.volatileSensorCallback)
        self.obstaclePublisher = rospy.Publisher("/{}/obstacle".format(name), Obstacle, queue_size=5)

    def laserScanCallback(self, data):
        self.obstacleAccumulator = Obstacle.PATH_IS_CLEAR
        self.obstacleMask = Obstacle.IS_LIDAR
        for i in range(0, len(data.ranges)):
            t = data.angle_min + (data.angle_increment * i)
            d = math.cos(t) * data.ranges[i]  # check these results
            w = math.sin(t) * data.ranges[i]  # check these results
            if (w < (self.rover_width / 2.0)) and (d < 1):  # @TODO: Fix MAGIC!
                self.obstacleAccumulator |= Obstacle.LIDAR_BLOCK

            if (i < (len(data.ranges) * self.laserOverlap)) and data.ranges[i] < 0.5:  # @TODO: Fix MAGIC!
                self.obstacleAccumulator |= Obstacle.LIDAR_LEFT

            if i < (len(data.ranges) * (1 / self.laserOverlap)) and (data.ranges[i] < 0.5):  # @TODO: Fix MAGIC!
                self.obstacleAccumulator |= Obstacle.LIDAR_RIGHT

            # @TODO: Fix MAGIC!
            if i > (len(data.ranges) * (self.laserOverlap / 2)) \
                    and (i < (len(data.ranges) * (self.laserOverlap * 2))
                         and data.ranges[i] < 0.5):  # @TODO: Fix MAGIC!
                self.obstacleAccumulator |= Obstacle.LIDAR_CENTER

            self.obstaclePublisher.publish(Obstacle(self.obstacleAccumulator,
                                               Obstacle.IS_LIDAR))
            '''['header', 'angle_min', 'angle_max', 'angle_increment', 'time_increment', 'scan_time', 'range_min', 'range_max', 'ranges', 'intensities'] '''

    def volatileSensorCallback(self, data):
        self.obstaclePublisher.publish(Obstacle(
            Obstacle.VOLATILE,
            Obstacle.IS_VOLATILE))

    def run(self):
        # @TODO: Test for laser noise if it's bad move publish into run after taking the median
        rospy.init_node('Obstacle', anonymous=True)
        rospy.loginfo("Obstacle Node Running")
        rospy.spin()


if __name__ == '__main__':
    rospy.loginfo("Obstacle Node Initializing")
    obstacleInstance = Obstacle("scout_1")
    try:
        obstacleInstance.run()
    except rospy.ROSInterruptException:
        pass
