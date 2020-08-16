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

		# generate and set the parameters for cv2 simple blob detection
		self.simple_blob_detector_params = cv2.SimpleBlobDetector_Params()
		self.simple_blob_detector_params.minThreshold = 1             # default = 1
		self.simple_blob_detector_params.maxThreshold = 100           # default = 255

		# filter by area
		self.simple_blob_detector_params.filterByArea = True          # default = True
		self.simple_blob_detector_params.minArea = 1000               # default = 1

		# filter by circularity
		self.simple_blob_detector_params.filterByCircularity = True   # default = False
		self.simple_blob_detector_params.minCircularity = 0.5

		# filter by convexity
		self.simple_blob_detector_params.filterByConvexity = True     # default = False
		self.simple_blob_detector_params.minConvexity = 0.25

		# filter by inertia
		self.simple_blob_detector_params.filterByInertia = False      # default = False
		self.simple_blob_detector_params.minInertiaRatio = 0.01

		self.simple_blob_detector_params.filterByColor = False         # default = False ?

		# create simple blob detector object
		self.detector = cv2.SimpleBlobDetector_create(self.simple_blob_detector_params)

		self.left_camera_subscriber = message_filters.Subscriber('/scout_1/camera/left/image_raw', Image)
		self.right_camera_subscriber = message_filters.Subscriber('/scout_1/camera/right/image_raw', Image)

		self.blob_detection_left_publisher = rospy.Publisher('/scout_1/blob_detections/left', Image, queue_size=10)
		self.blob_detection_right_publisher = rospy.Publisher('/scout_1/blob_detections/right', Image, queue_size=10)

		self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.left_camera_subscriber, self.right_camera_subscriber], 10, 0.1, allow_headerless=True)
		self.synchronizer.registerCallback(self.callback)


	def callback(self, left_camera_data, right_camera_data):
		# left_camera_data and right_camera_data are sensor_msg/Image data types

		# convert image data from image message -> opencv image
		cv_image_left = cv2.cvtColor(self.bridge.imgmsg_to_cv2(left_camera_data, desired_encoding="passthrough"), cv2.COLOR_BGR2RGB)
		cv_image_right = cv2.cvtColor(self.bridge.imgmsg_to_cv2(right_camera_data, desired_encoding="passthrough"), cv2.COLOR_BGR2RGB)

		# generate the blob detections
		keypoints_left = self.detector.detect(cv_image_left)
		keypoints_right = self.detector.detect(cv_image_right)

		# super-impose the blob detections over the original camera image
		im_with_keypoints_left = cv2.drawKeypoints(cv_image_left, keypoints_left, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		im_with_keypoints_right = cv2.drawKeypoints(cv_image_right, keypoints_right, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

		# convert cv2 images into ros messages and publish
		imgmsg_left = self.bridge.cv2_to_imgmsg(im_with_keypoints_left, encoding="passthrough")
		imgmsg_right = self.bridge.cv2_to_imgmsg(im_with_keypoints_right, encoding="passthrough")
		self.blob_detection_left_publisher.publish(imgmsg_left)
		self.blob_detection_right_publisher.publish(imgmsg_right)


if __name__ == '__main__':
	rospy.init_node('scoot_object_detection')
	ObjectDetection()
	rospy.spin()

