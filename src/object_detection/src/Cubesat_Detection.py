#!/usr/bin/env python

from __future__ import division
import rospy
import message_filters
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.point_cloud2 import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
import time
import imutils
import math
from scipy.spatial import distance as dist
from collections import OrderedDict
from object_detection.msg import Detection


class CubesatDetection(object):

	def __init__(self):
		# note: required library "pip install imutils"

		self.bridge = CvBridge()

		self.point_cloud_subscriber = message_filters.Subscriber('/scout_1/points2', PointCloud2)
		self.left_camera_subscriber = message_filters.Subscriber('/scout_1/camera/left/image_raw', Image)

		self.cubesat_detection_image_left_publisher = rospy.Publisher('/scout_1/cubesat_detections/image/left', Image, queue_size=10)
		self.cubesat_detection_left_publisher = rospy.Publisher('/scout_1/cubesat_detections/', Detection, queue_size=10)

		self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.left_camera_subscriber], 10, 0.1, allow_headerless=True)
		self.synchronizer.registerCallback(self.callback)

		self.colors_blue = OrderedDict()
		
		for i in range(1, 256):
			temp = {"blue_" + str(i) : (0,0,255)}
			self.colors_blue.update(temp)
		# colors = OrderedDict({"red": (255, 0, 0),"green": (0, 255, 0),"blue": (0, 0, 255)})
		
		self.lab = np.zeros((len(self.colors_blue), 1, 3), dtype="uint8")
		self.colorNames = []
		for (i, (name, rgb)) in enumerate(self.colors_blue.items()):
			# update the L*a*b* array and the color names list
			self.lab[i] = rgb
			self.colorNames.append(name)

		# convert the L*a*b* array from the RGB color space
		# to L*a*b*
		self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2LAB)

		# subscribe to camera_info topics to get focal lengths and other camera settings/data as needed
		self.left_camera_info_subscriber = message_filters.Subscriber('/scout_1/camera/left/camera_info', CameraInfo)
		self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.left_camera_info_subscriber], 10, 0.1, allow_headerless=True)
		self.synchronizer.registerCallback(self.camera_info_callback)
		self.left_camera_focal_length = 380.0


	def camera_info_callback(self, left_camera_info):
		self.left_camera_focal_length = left_camera_info.K[0]


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


	def callback(self, left_camera_data):
		left_detection_msg = Detection()
		left_detection_msg.detection_id = 999 #"Cubesat Detection" TODO: change this
		left_detection_msg.left_heading = 0.0
		left_detection_msg.left_distance = 0.0

		# convert image data from image message -> opencv image
		cv_image_left = cv2.cvtColor(self.bridge.imgmsg_to_cv2(left_camera_data, desired_encoding="passthrough"), cv2.COLOR_BGR2RGB)

		# determine colors and generate shapes			
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
			if M["m00"] != 0:
				cX = int((M["m10"] / M["m00"]))
				cY = int((M["m01"] / M["m00"]))
				area = cv2.contourArea(c)
				if area > 300 and area < 1500 : 
					shape = self.detect(c)
					color = self.label(lab_left,c)
					# print(color)
				
					if shape == 'rectangle':
						c = c.astype("float")
						c *= ratio_left
						c = c.astype("int")
						cv2.drawContours(cv_image_left, [c], -1, (0, 255, 0), 3)
						marker = cv2.minAreaRect(c)
						focalLength= self.left_camera_focal_length
						KNOWN_WIDTH = 1 # cubesat width in meters
						per_width= marker[1][0]
						distance_meters = self.distance_to_camera(KNOWN_WIDTH, focalLength, per_width)
						left_detection_msg.left_distance = distance_meters					
						print(str(distance_meters) + ' meters')
						print('X = ' + str(cX) + ' Y = ' + str(cY))

						angle = (math.pi / 2) - ((cY / 480) * math.pi / 2)
						left_detection_msg.left_heading = ((cX - 320) / 640) * 2.0944 # radians (approx 120 degrees)
						camera_offset_from_ground = 0.5
						z = (distance_meters * math.sin(angle)) + camera_offset_from_ground

						print('\nheading? = ' + str(heading))
						print('z? = ' + str(z))

						# x = cX;
						# y = cY * point_cloud_msg.width
						# index = x + y
						# print(points_list[index])

						self.cubesat_detection_left_publisher.publish(left_detection_msg)

		imgmsg_left = self.bridge.cv2_to_imgmsg(cv_image_left, encoding="passthrough")
		self.cubesat_detection_image_left_publisher.publish(imgmsg_left)

