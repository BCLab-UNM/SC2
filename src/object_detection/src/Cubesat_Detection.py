#!/usr/bin/env python

from __future__ import division
import rospy
import message_filters
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.point_cloud2 import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from obstacle.msg import Obstacles
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
import time
import imutils
import math
from scipy.spatial import distance as dist
from scipy.spatial.transform import Rotation
from collections import OrderedDict
from object_detection.msg import Detection
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
# required libraries:
#     - sudo apt-get install ros-melodic-geometry2
#     - pip install imutils
#     - pip install scipy==1.2.0


class CubesatDetection(object):

	def __init__(self):
		self.bridge = CvBridge()

		self.point_cloud_subscriber = rospy.Subscriber('/scout_1/points2', PointCloud2, self.pc_callback)
		self.scoot_odom_subscriber = rospy.Subscriber('/scout_1/odom/filtered', Odometry, self.odom_callback)
		self.left_camera_subscriber = message_filters.Subscriber('/scout_1/camera/left/image_raw', Image)

		self.cubesat_detection_image_left_publisher = rospy.Publisher('/scout_1/cubesat_detections/image/left', Image, queue_size=10)
		self.cubesat_detection_publisher = rospy.Publisher('/scout_1/cubesat_detections/', Detection, queue_size=10)

		self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.left_camera_subscriber], 10, 0.1, allow_headerless=True)
		self.synchronizer.registerCallback(self.callback)

		self.colors_blue = OrderedDict()

		# self.z_value_list = []
		
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

		# convert the L*a*b* array from the RGB color space to L*a*b*
		self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2LAB)

		# subscribe to camera_info topics to get focal lengths and other camera settings/data as needed
		self.left_camera_info_subscriber = message_filters.Subscriber('/scout_1/camera/left/camera_info', CameraInfo)
		self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.left_camera_info_subscriber], 10, 0.1, allow_headerless=True)
		self.synchronizer.registerCallback(self.camera_info_callback)
		self.left_camera_focal_length = 380.0
		
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) # tf buffer length
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		self.heading = None
		self.distance = None
		self.odom_pose = None
		self.detection_pose = None


	def odom_callback(self, odom_msg):
		# extract the robot's XYZ position and heading (q) from the odometry message
		self.odom_pose = [0, 0, 0]
		self.odom_pose[0] = odom_msg.pose.pose.position.x
		self.odom_pose[1] = odom_msg.pose.pose.position.y
		self.odom_pose[2] = odom_msg.pose.pose.position.z
		q = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
		h = Rotation.from_quat(q)
		self.heading = (h.as_rotvec())[2]
		# print('    xyz = ' + str(self.odom_pose))
		# print('heading = ' + str(self.heading))


	def pc_callback(self, point_cloud_msg):
		'''
		This callback function takes point cloud data and along with a TF transform
		to the robot base link calculates an approximate xyz coordinate for the cubesat

		assumptions:
		    - we are pointing the camera up @ 0.78 radians
		    - only the cubesat is visible in the sky
		    - we have access to accurate odometry
		'''

		# -------------------------------------------------------------
		# take the average XYZ point of all point cloud points detected
		# -------------------------------------------------------------
		x_sum = 0.0
		y_sum = 0.0
		z_sum = 0.0
		count = 0.0		

		for data in pc2.read_points(point_cloud_msg, skip_nans=True):
			x_sum += data[0]
			y_sum += data[1]
			z_sum += data[2]
			count += 1

		if count < 1:
			print('no point cloud detection')
			return
		else:
			x_avg = x_sum / count
			y_avg = y_sum / count
			z_avg = z_sum / count
			# print('number of points = ' + str(count))
			# print('point cloud average: x = ' + str(x) + ', y = ' + str(y) + ', z = ' + str(z))

		H = z_avg # hypotenuse of a right triangle from scout to cubesat (triangle HZV)
		self.distance = H

		# ----------------------------------------------------------------------------------------------------------
		# calculate a transform from the point cloud to our camera frame of reference
		# ONLY THE Z VALUE IS ACCURATE IN THIS TRANSFORM, further processing is needed to get the X and Y using odom
		# ----------------------------------------------------------------------------------------------------------
		transform = self.tf_buffer.lookup_transform('scout_1_tf/base_footprint', point_cloud_msg.header.frame_id, point_cloud_msg.header.stamp, rospy.Duration(2.0))
		pose_stamped = PoseStamped()
		pose_stamped.header = point_cloud_msg.header
		pose_stamped.pose.position.x = x_avg # point[0]
		pose_stamped.pose.position.y = y_avg # point[1]
		pose_stamped.pose.position.z = z_avg # point[2]
		pose_stamped.pose.orientation = transform.transform.rotation
		pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
		
		Z = pose_transformed.pose.position.z # side Z of a right triangle from scout to cubesat (triangle HZV)
		
		V = (H * H) - (Z * Z) # size V of a right triangle from scout to cubesat (triangle HZV)
                                      # pythagorean thereom: V^2 = H^2 - Z^2; V = sqrt(H^2 - Z^2)
		
		# consider some edge cases for the calculation of V and make an estimate adjustment
		if (V < 0.0):
			V *= -1.0;
		else:
			V = math.sqrt(V)

		if self.heading != None:
			# V is the magnitude of a vector from the robot to the cubesat, we can use trigonometry to approximate a transform
			# from the XY of the robot to the XY of the cubesat (Z is already calculated above)
			self.detection_pose = [0, 0, 0]
			self.detection_pose[2] = Z + self.odom_pose[2]
			self.detection_pose[1] = (V * math.sin(self.heading - self.heading_correction)) + self.odom_pose[1]
			self.detection_pose[0] = (V * math.cos(self.heading - self.heading_correction)) + self.odom_pose[0]
			print(self.detection_pose)


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
		detection_msg = Detection()
		detection_msg.detection_id = Obstacles.CUBESAT
		detection_msg.heading = None
		detection_msg.distance = None
		detection_msg.x = None
		detection_msg.y = None
		detection_msg.z = None

		# convert image data from image message -> opencv image
		cv_image_left = cv2.cvtColor(self.bridge.imgmsg_to_cv2(left_camera_data, desired_encoding="passthrough"), cv2.COLOR_BGR2RGB)

		# determine colors and generate shapes			
		resized_left = imutils.resize(cv_image_left, width=640)
		ratio_left = resized_left.shape[0] / float(resized_left.shape[0])
		gray_left = cv2.cvtColor(resized_left, cv2.COLOR_BGR2GRAY)
		lab_left = cv2.cvtColor(resized_left, cv2.COLOR_BGR2LAB)
		blurred_left = cv2.GaussianBlur(gray_left, (5, 5), 0)
		thresh_left = cv2.threshold(blurred_left, 110, 255, cv2.THRESH_BINARY)[1]
		cnts_left = cv2.findContours(thresh_left.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts_left = imutils.grab_contours(cnts_left)

		for c in cnts_left:
			M = cv2.moments(c)
			if M["m00"] != 0:
				cX = int((M["m10"] / M["m00"])  * ratio_left)
				cY = int((M["m01"] / M["m00"])  * ratio_left)
				area = cv2.contourArea(c)
				if area > 50 and area < 1500:
					shape = self.detect(c)
					color = self.label(lab_left,c)
					if shape == 'rectangle':
						c = c.astype("float")
						c *= ratio_left
						c = c.astype("int")
						cv2.drawContours(cv_image_left, [c], -1, (0, 255, 0), 3)
						marker = cv2.minAreaRect(c)
						focalLength = self.left_camera_focal_length
						KNOWN_WIDTH = 1 # cubesat width in meters
						per_width = marker[1][0]
						if self.detection_pose != None:
							detection_msg.x = self.detection_pose[0]
							detection_msg.y = self.detection_pose[1]
							detection_msg.z = self.detection_pose[2]
							detection_msg.distance = self.distance
							detection_msg.heading = ((cX - 320) / 640) * 2.0944 # radians (approx 120 degrees)
							self.detection_pose = None
							self.cubesat_detection_publisher.publish(detection_msg)

		# publish the detection image topic showing the detected object contour
		# used for debugging and visualisation
		imgmsg_left = self.bridge.cv2_to_imgmsg(cv_image_left, encoding="passthrough")
		self.cubesat_detection_image_left_publisher.publish(imgmsg_left)

