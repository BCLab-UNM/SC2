#!/usr/bin/env python


import rospy
import message_filters
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class DisparityMap(object):

	def __init__(self):
		self.left_camera_subscriber = message_filters.Subscriber('/scout_1/camera/left/image_raw', Image)
		self.right_camera_subscriber = message_filters.Subscriber('/scout_1/camera/right/image_raw', Image)

		# need to also create a publisher
		# this is an example, and needs to be updated
		self.disparity_map_publisher = rospy.Publisher('/test', String, queue_size=10)

		self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.left_camera_subscriber, self.right_camera_subscriber], 10, 0.1, allow_headerless=True)
		self.synchronizer.registerCallback(self.callback)

	def callback(self, left_camera_data, right_camera_data):
		# left_camera_data and right_camera_data are sensor_msg/Image data types

		# code goes here
		######

		self.disparity_map_publisher.publish('test')


if __name__ == '__main__':
	rospy.init_node('disparityMap')
	DisparityMap()
	rospy.spin()

