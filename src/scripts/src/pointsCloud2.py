#!/usr/bin/env python


import rospy
import ros_numpy
from sensor_msgs.point_cloud2 import PointCloud2


class Points2(object):

	def __init__(self):
		self.points2_subscriber = rospy.Subscriber('/scout_1/points2', PointCloud2, self.callback)


	def callback(self, pointcloud2_msg):
		# sudo apt install ros-melodic-ros-numpy
		xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pointcloud2_msg)
		print(xyz_array.shape)


if __name__ == '__main__':
	rospy.init_node('points_cloud_2')
	Points2()
	rospy.spin()

