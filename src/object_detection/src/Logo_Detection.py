#!/usr/bin/env python


import rospy
import message_filters
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
import time
import imutils
from scipy.spatial import distance as dist
from collections import OrderedDict

# the message type for our publisher is Detection
#     it has 3 fields:
#         - detection_id:  the string name of the object we have detected
#         - heading:       the heading we need to turn to face our detected object in radians
#         - distance:      the distance to the detected object in metres
from object_detection.msg import Detection


class LogoDetection(object):

	def __init__(self):
		# note: required library "pip install imutils"

		self.bridge = CvBridge()

		self.left_camera_subscriber = message_filters.Subscriber('/scout_1/camera/left/image_raw', Image)
		self.right_camera_subscriber = message_filters.Subscriber('/scout_1/camera/right/image_raw', Image)

		self.logo_detection_left_publisher = rospy.Publisher('/scout_1/logo_detections/left', Image, queue_size=10)
		self.logo_detection_right_publisher = rospy.Publisher('/scout_1/logo_detections/right', Image, queue_size=10)

		self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.left_camera_subscriber, self.right_camera_subscriber], 10, 0.1, allow_headerless=True)
		self.synchronizer.registerCallback(self.callback)

		colors = OrderedDict({"red": (255, 0, 0),"green": (0, 255, 0),"blue": (0, 0, 255)})
		
		self.lab = np.zeros((len(colors), 1, 3), dtype="uint8")
		self.colorNames = []
		for (i, (name, rgb)) in enumerate(colors.items()):
			# update the L*a*b* array and the color names list
			self.lab[i] = rgb
			self.colorNames.append(name)

		# convert the L*a*b* array from the RGB color space
		# to L*a*b*
		self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2LAB)

		# subscribe to camera_info topics to get focal lengths and other camera settings/data as needed
		self.left_camera_info_subscriber = message_filters.Subscriber('/scout_1/camera/left/camera_info', CameraInfo)
		self.right_camera_info_subscriber = message_filters.Subscriber('/scout_1/camera/right/camera_info', CameraInfo)
		self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.left_camera_info_subscriber, self.right_camera_info_subscriber], 10, 0.1, allow_headerless=True)
		self.synchronizer.registerCallback(self.camera_info_callback)
		self.left_camera_focal_length = 380.0
		self.right_camera_focal_length = 380.0


	def camera_info_callback(self, left_camera_info, right_camera_info):
		self.left_camera_focal_length = left_camera_info.K[0]
		self.right_camera_focal_length = right_camera_info.K[4]


	def detect(self, c):
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)

		if len(approx) == 3:
			shape = "triangle"
		elif len(approx) == 4:
			shape = "rectangle"
		elif len(approx) == 5:
			shape = "pentagon"
		else:
			shape = "circle"

		return shape

	def label(self, image, c):
		mask = np.zeros(image.shape[:2], dtype="uint8")
		cv2.drawContours(mask, [c], -1, 255, -1)
		mask = cv2.erode(mask, None, iterations=2)
		mean = cv2.mean(image, mask=mask)[:3]
		# initialize the minimum distance found thus far
		minDist = (np.inf, None)
		# loop over the known L*a*b* color values
		for (i, row) in enumerate(self.lab):
			# compute the distance between the current L*a*b*
			# color value and the mean of the image
			d = dist.euclidean(row[0], mean)
			# if the distance is smaller than the current distance,
			# then update the bookkeeping variable
			if d < minDist[0]:
				minDist = (d, i)
		# return the name of the color with the smallest distance
		return self.colorNames[minDist[1]]

	def distance_to_camera(self, knownWidth, focalLength, perWidth):
		# compute and return the distance from the maker to the camera
		return (knownWidth * focalLength) / perWidth


	def callback(self, left_camera_data, right_camera_data):
		# left_camera_data and right_camera_data are sensor_msg/Image data types

		# convert image data from image message -> opencv image
		cv_image_left = cv2.cvtColor(self.bridge.imgmsg_to_cv2(left_camera_data, desired_encoding="passthrough"), cv2.COLOR_BGR2RGB)
		cv_image_right = cv2.cvtColor(self.bridge.imgmsg_to_cv2(right_camera_data, desired_encoding="passthrough"), cv2.COLOR_BGR2RGB)

		#determine colors

		#generate shapes			
		# TODO: re-write for left and right camera
		resized_left = imutils.resize(cv_image_left, width=640)
		ratio_left = resized_left.shape[0] / float(resized_left.shape[0])
		gray_left = cv2.cvtColor(resized_left, cv2.COLOR_BGR2GRAY)
		lab_left = cv2.cvtColor(resized_left, cv2.COLOR_BGR2LAB)
		blurred_left = cv2.GaussianBlur(gray_left, (5, 5), 0)
		thresh_left = cv2.threshold(blurred_left, 110, 255, cv2.THRESH_BINARY)[1]
		#thresh_left = thresh_left.astype(np.uint8)
		cnts_left = cv2.findContours(thresh_left.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts_left = imutils.grab_contours(cnts_left)

		for c in cnts_left:
			M = cv2.moments(c)
			#cX = int((M["m10"] / M["m00"]) * ratio_left)
			#cY = int((M["m01"] / M["m00"]) * ratio_left)
			area = cv2.contourArea(c)
			if area > 300 and area < 1500 : 
				shape = self.detect(c)
				color = self.label(lab_left,c)
			#print(color)
				if shape== 'triangle' or color == 'blue':
					c = c.astype("float")
					c *= ratio_left
					c = c.astype("int")
					cv2.drawContours(cv_image_left, [c], -1, (0, 255, 0), 3)
					#print(M['m10'])
					marker = cv2.minAreaRect(c)
					focalLength= self.left_camera_focal_length
					KNOWN_WIDTH = 2.72 #logo width in inches
					per_width= marker[1][0]
					inches = self.distance_to_camera(KNOWN_WIDTH, focalLength, per_width)
					print(inches)

				#cv2.putText(cv_image_left, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

		#resized_right = imutils.resize(cv_image_right, width=300)
		#ratio_right = resized_right.shape[0] / float(resized_right.shape[0])
		#gray_right = cv2.cvtColor(resized_right, cv2.COLOR_BGR2GRAY)
		#blurred_right = cv2.GaussianBlur(gray_right, (5, 5), 0)
		#thresh_right = cv2.threshold(blurred_right, 60, 255, cv2.THRESH_BINARY)[1]
		#thresh_right = thresh_right.astype(np.uint8)
		#cnts_right = cv2.findContours(thresh_right.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		#cnts_right = imutils.grab_contours(cnts_right)

		#for c in cnts_right:
		#	M = cv2.moments(c)
			#cX = int((M["m10"] / M["m00"]) * ratio_right)
			#cY = int((M["m01"] / M["m00"]) * ratio_right)
		#	shape = self.detect(c)
		#	if shape is "rectangle":
		#		c = c.astype("int")
		#		cv2.drawContours(cv_image_right, [c], -1, (0, 255, 0), 2)

		imgmsg_left = self.bridge.cv2_to_imgmsg(cv_image_left, encoding="passthrough")
		#imgmsg_right = self.bridge.cv2_to_imgmsg(cv_image_right, encoding="passthrough")
		self.logo_detection_left_publisher.publish(imgmsg_left)
		#self.logo_detection_right_publisher.publish(imgmsg_right)

