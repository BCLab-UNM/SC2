#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from srcp2_msgs.msg import VolSensorMsg
from obstacle.msg import Obstacles

# @TODO: check/test for logic errors someone else

class Obstacle:

    def __init__(self):
        rospy.init_node('Obstacle', anonymous=True)
        self.name = rospy.get_namespace()
        self.name = self.name.strip('/')
        self.rover_width = rospy.get_param('/rover_total_width', default=2.2098)
        self.laser_coverage = rospy.get_param('/laser_coverage', default=40)
        self.rover_width = rospy.get_param('/rover_total_width', default=2.2098)
        self.safe_distance = rospy.get_param('/safe_distance', default=0.7)
        self.warning_distance = rospy.get_param('/warning_distance', default=1.1)
        self.laserScan = LaserScan()
        self.closest = float('inf')
        self.volSensor = VolSensorMsg()
        self.obstacleAccumulator = Obstacles.PATH_IS_CLEAR
        self.obstacleMask = Obstacles.PATH_IS_CLEAR
        rospy.Subscriber("/{}/laser/scan".format(self.name), LaserScan, self.laser_scan_callback)
        rospy.Subscriber("/{}/volatile_sensor".format(self.name), VolSensorMsg, self.volatile_sensor_callback)
        self.obstaclePublisher = rospy.Publisher("/{}/obstacle".format(self.name), Obstacles, queue_size=5)
        self.VOL_TYPES = rospy.get_param("vol_types", default=["ice", "ethane", "methane", "methanol", "carbon_dioxide", "ammonia", "hydrogen_sulfite", "sulfur_dioxide", "regolith"])

    def laser_scan_callback(self, data):
        self.obstacleAccumulator = Obstacles.PATH_IS_CLEAR
        self.obstacleMask = Obstacles.IS_LIDAR
        for i in range(0, len(data.ranges)):
            theta = data.angle_min + (data.angle_increment * i)
            d = math.cos(theta) * data.ranges[i]  # check these results
            w = math.sin(theta) * data.ranges[i]  # check these results
            # If the rover was driving forward would hit something within than safe_distance
            if (w < (self.rover_width / 2.0)) and (d < self.safe_distance):
                self.obstacleAccumulator |= Obstacles.LIDAR_BLOCK
                if self.closest > d:
                    self.closest = d

            # laser_coverage defines what percentage of the total scanned area is assigned to each subarea: LIDAR_LEFT, LIDAR_RIGHT, LIDAR_CENTER
            # so if laser_coverage was 45, as in 45%, and the sensor took 1000 readings (len(data.ranges) == 1000)
            # then coverage_value would be 450, so LIDAR_RIGHT would be comprised of indexes 0->450 in the data.range
            coverage_value = self.laser_coverage / 100.0 * len(data.ranges)

            # Blocked right side of Lidar within warning_distance
            if i < coverage_value and data.ranges[i] < self.warning_distance:
                self.obstacleAccumulator |= Obstacles.LIDAR_RIGHT
                if self.closest > data.ranges[i]:
                    self.closest = d

            # Blocked left side of Lidar within warning_distance
            if i > (len(data.ranges) - coverage_value) and data.ranges[i] < self.warning_distance:
                self.obstacleAccumulator |= Obstacles.LIDAR_LEFT
                if self.closest > data.ranges[i]:
                    self.closest = d

            # Blocked center
            # if range within right side
            # if range within left side
            # if range within warning_distance
            if (i > ((len(data.ranges) / 2) - (self.laser_coverage / 2))) and \
                    (i < ((len(data.ranges) / 2) + (self.laser_coverage / 2)) and
                     data.ranges[i] < self.warning_distance):
                self.obstacleAccumulator |= Obstacles.LIDAR_CENTER
                if self.closest > data.ranges[i]:
                    self.closest = d

            self.obstaclePublisher.publish(Obstacles(self.obstacleAccumulator,
                                                     Obstacles.IS_LIDAR, 0, self.closest))
            self.closest = float('inf')

    def volatile_sensor_callback(self, data):
        if data.distance_to > 0:
            self.obstaclePublisher.publish(Obstacles(
                Obstacles.VOLATILE,
                Obstacles.IS_VOLATILE,
                self.VOL_TYPES.index(data.vol_type),
                0))

    def run(self):
        # @TODO: Test for laser noise if it's bad move publish into run after taking the median
        rospy.loginfo("Obstacle Node Running")
        rospy.spin()


if __name__ == '__main__':
    rospy.loginfo("Obstacle Node Initializing")
    obstacleInstance = Obstacle()
    try:
        obstacleInstance.run()
    except rospy.ROSInterruptException:
        pass
