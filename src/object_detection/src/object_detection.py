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


class ObjectDetection(object):

	def __init__(self):
		self.bridge = CvBridge()

		self.left_camera_subscriber = message_filters.Subscriber('/scout_1/camera/left/image_raw', Image)
		self.right_camera_subscriber = message_filters.Subscriber('/scout_1/camera/right/image_raw', Image)

                # TODO: we need to add a different publisher later
		# self.disparity_map_publisher = rospy.Publisher('/scout_1/camera/disparity', Image, queue_size=10)

		self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.left_camera_subscriber, self.right_camera_subscriber], 10, 0.1, allow_headerless=True)
		self.synchronizer.registerCallback(self.callback)


	def callback(self, left_camera_data, right_camera_data):
		# left_camera_data and right_camera_data are sensor_msg/Image data types

		# convert image data from image message -> opencv image -> numpy array
		cv_image_left = self.bridge.imgmsg_to_cv2(left_camera_data, desired_encoding="mono8")
		# np_image_left = np.asarray(cv_image_left)	
		cv_image_right = self.bridge.imgmsg_to_cv2(right_camera_data, desired_encoding="mono8")
		# np_image_right = np.asarray(cv_image_right)

		# convert our disparity array into a cv2 image and publish
		# disparity_cv2 = cv2.normalize(src=disparity, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
		# imgmsg = self.bridge.cv2_to_imgmsg(disparity_cv2, encoding="passthrough")
		# self.disparity_map_publisher.publish(imgmsg)


if __name__ == '__main__':
	rospy.init_node('scoot_object_detection')
	ObjectDetection()
	rospy.spin()

