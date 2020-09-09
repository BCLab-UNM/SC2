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
from obstacle.msg import Obstacles
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
import time
import imutils
import math
from scipy.spatial import distance as dist
from collections import OrderedDict
from object_detection.msg import Detection
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped



class LogoDetection(object):

	def __init__(self):
		# note: required library "pip install imutils"

		self.bridge = CvBridge()

		self.point_cloud_subscriber = rospy.Subscriber('/scout_1/points2', PointCloud2, self.pc_callback)
		self.left_camera_subscriber = message_filters.Subscriber('/scout_1/camera/left/image_raw', Image)
		

		self.logo_detection_image_left_publisher = rospy.Publisher('/scout_1/logo_detections/image/left', Image, queue_size=10)
		self.logo_detection_left_publisher = rospy.Publisher('/scout_1/object_detections', Detection, queue_size=10)

		self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.left_camera_subscriber], 10, 0.1, allow_headerless=True)
		self.synchronizer.registerCallback(self.callback)

		self.z_value_list = []


		# self.colors_blue = OrderedDict()
		# self.colors_blue.update( {"red" : (255,0,0)})
		# self.colors_blue.update( {"blue" : (0,0,255)})

		# for i in range(1, 256):
			# temp = {"blue_" + str(i) : (0,0,i)}
			# self.colors_blue.update(temp)
		colors = OrderedDict({"blue": (0, 0, 255),"red": (255, 0, 0), "green": (0, 255, 0)})
		
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

		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) # tf buffer length
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		self.pose_transformed = None
		self.heading = None
		

	def pc_callback(self, point_cloud_msg):
		points_list = []
		
		for data in pc2.read_points(point_cloud_msg, skip_nans=True):
			points_list.append([data[0], data[1], data[2]])

		if len(points_list) == 0:
			print('no point cloud')
			return

		# scout_1_tf/base_footprint
		try:
			transform = self.tf_buffer.lookup_transform('scout_1_tf/base_footprint', point_cloud_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

			index = int(len(points_list)/2)
			point = points_list[index]
			pose_stamped = PoseStamped()
			pose_stamped.header = point_cloud_msg.header
			pose_stamped.pose.position.x = point[0] # see stereo_image_proc docs, the xyz need to be remapped
			pose_stamped.pose.position.y = point[1]
			pose_stamped.pose.position.z = point[2]
			# pose_stamped.pose.orientation = transform.pose.orientation
			
			pre_pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
			self.pose_transformed = PoseStamped()
			self.pose_transformed.header = pre_pose_transformed.header
			self.pose_transformed.pose.position.x = pre_pose_transformed.pose.position.x
			self.pose_transformed.pose.position.y = pre_pose_transformed.pose.position.y
			self.pose_transformed.pose.position.z = pre_pose_transformed.pose.position.z
			self.pose_transformed.orientation = pre_pose_transformed.orientation
			# self.pose_transformed = pre_pose_transformed

		except Exception:
			# self.pose_transformed = None
			return

	def camera_info_callback(self, left_camera_info, right_camera_info):
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
		left_detection_msg.detection_id = Obstacles.HOME_FIDUCIAL # this is an integer ID defined in the obstacle package
		left_detection_msg.left_heading = 0.0
		left_detection_msg.left_distance = 0.0
		

		# left_camera_data and right_camera_data are sensor_msg/Image data types

		# convert image data from image message -> opencv image
		cv_image_left = cv2.cvtColor(self.bridge.imgmsg_to_cv2(left_camera_data, desired_encoding="passthrough"), cv2.COLOR_BGR2RGB)
		
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

			if M["m00"] != 0:
				cX = int((M["m10"] / M["m00"]) * ratio_left)
				cY = int((M["m01"] / M["m00"]) * ratio_left)
				area = cv2.contourArea(c)
				if area > 300 and area < 1000 : 
					shape = self.detect(c)
					color = self.label(lab_left,c)
					print(color)
				
					# if color == 'red':
					# if shape == 'triangle':
						c = c.astype("float")
						c *= ratio_left
						c = c.astype("int")
						cv2.drawContours(cv_image_left, [c], -1, (0, 255, 0), 3)
						#print(M['m10'])
						marker = cv2.minAreaRect(c)
						focalLength= self.left_camera_focal_length
						KNOWN_WIDTH = 0.955 #logo width in meterswith
						per_width= marker[1][0]
						distance_meters = self.distance_to_camera(KNOWN_WIDTH, focalLength, per_width)
						left_detection_msg.left_distance = distance_meters					
						print(str(distance_meters) + ' meters')
						print('X = ' + str(cX) + ' Y = ' + str(cY))
						left_detection_msg.left_heading = ((cX - 320) / 640) * 2.0944 # radians (approx 120 degrees)
						y_heading = (((cY - 240) / 480) * 2.0944) 

						camera_offset_from_ground = 0.5
					 

						z = ( distance_meters * math.sin(0.0)) + camera_offset_from_ground

						y_pos =  ( distance_meters *  math.sin(left_detection_msg.left_heading))+ camera_offset_from_ground
						x_pos = ( distance_meters * math.sin(y_heading))+ camera_offset_from_ground 

						self.z_value_list.append(z)

						if (len(self.z_value_list)>=20):
							self.z_value_list.pop(0)

						z_average = sum(self.z_value_list) / len(self.z_value_list)

						if self.pose_transformed != None:
							print(self.pose_transformed)
							# print(type(self.pose_transformed))
							self.pose_transformed = None

						# print('angle '+ str(y_angle+x_angle + 0.78)) 
						print('headingX? = ' + str(left_detection_msg.left_heading))
						print('headingY? = ' + str(y_heading))
						print('x? = ' + str(x_pos))
						print('y? = ' + str(y_pos))
						print('z? = ' + str(z_average))


						self.logo_detection_left_publisher.publish(left_detection_msg)
	
				#cv2.putText(cv_image_left, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

		

		imgmsg_left = self.bridge.cv2_to_imgmsg(cv_image_left, encoding="passthrough")

		self.logo_detection_image_left_publisher.publish(imgmsg_left)
		
