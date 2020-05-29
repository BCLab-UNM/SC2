#!/usr/bin/env python


import rospy
import message_filters
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
import time

class DisparityMap(object):

	def __init__(self):
		self.stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
		self.bridge = CvBridge()

		self.left_camera_subscriber = message_filters.Subscriber('/scout_1/camera/left/image_raw', Image)
		self.right_camera_subscriber = message_filters.Subscriber('/scout_1/camera/right/image_raw', Image)

		# need to also create a publisher
		# this is an example, and needs to be updated
		self.disparity_map_publisher = rospy.Publisher('/test', String, queue_size=10)

		self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.left_camera_subscriber, self.right_camera_subscriber], 10, 0.1, allow_headerless=True)
		self.synchronizer.registerCallback(self.callback)


	def callback(self, left_camera_data, right_camera_data):
		# left_camera_data and right_camera_data are sensor_msg/Image data types

		# convert image data from image message -> opencv image -> numpy array
		cv_image_left = self.bridge.imgmsg_to_cv2(left_camera_data, desired_encoding="mono8")
		np_image_left = np.asarray(cv_image_left)
	
		cv_image_right = self.bridge.imgmsg_to_cv2(right_camera_data, desired_encoding="mono8")
		np_image_right = np.asarray(cv_image_right)

		disparity = self.stereo.compute(np_image_left, np_image_right)
		
		plt.imshow(np_image_left, 'gray')
		plt.figure()
		plt.imshow(np_image_right, 'gray')
		plt.figure()
		plt.imshow(disparity, 'gray')
		plt.show()
		
		#self.disparity_map_publisher.publish('test')


if __name__ == '__main__':
	rospy.init_node('disparityMap')
	DisparityMap()
	rospy.spin()

