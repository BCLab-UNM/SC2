#!/usr/bin/env python3

from __future__ import division
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.point_cloud2 import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from obstacle.msg import Obstacles
from cv_bridge import CvBridge
import imutils
import math
from scipy.spatial import distance as dist
from scipy.spatial.transform import Rotation
from collections import OrderedDict
from object_detection.msg import Detection
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from srcp2_msgs import msg, srv
from rospy import ServiceException
# required libraries:
#     - sudo apt-get install ros-melodic-geometry2
#     - pip install imutils
#     - pip install scipy==1.2.0


class CubesatDetection(object):

	def __init__(self):
		
		rospy.logwarn('from cubesat')
		self.bridge = CvBridge()

		self.point_cloud_subscriber = rospy.Subscriber('/small_scout_1/points2', PointCloud2, self.pc_callback)
		# self.scoot_odom_subscriber = rospy.Subscriber('/small_scout_1/odometry/filtered', Odometry, self.odom_callback)
		self.left_camera_subscriber = rospy.Subscriber('/small_scout_1/camera/left/image_raw', Image, self.cam_callback)

		self.cubesat_detection_image_left_publisher = rospy.Publisher('/small_scout_1/cubesat_detections/image/left', Image, queue_size=100)
		self.cubesat_detection_publisher = rospy.Publisher('/small_scout_1/detections/', Detection, queue_size=100)
		
		colors = OrderedDict({"yellow": (255, 195, 0), "red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255), "white": (255, 255, 255), "black": (0, 0, 0),})
		self.lab = np.zeros((len(colors), 1, 3), dtype="uint8")
		self.colorNames = []
		for (i, (name, rgb)) in enumerate(colors.items()):
			# update the L*a*b* array and the color names list
			self.lab[i] = rgb
			self.colorNames.append(name)

		# convert the L*a*b* array from the RGB color space to L*a*b*
		self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2LAB)

		# transform variables for getting TF data for use in point cloud transform
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		# output & supporting variables for detection message publisher
		self.heading = None
		self.distance = None
		self.odom_pose = None
		self.detection_pose = None
		self.heading_correction = 0.0
		self.debug = True
		self.used_truepose = False


	def get_truepose(self):
		# extract the robot's XYZ position and heading (q) from the odometry message
		rospy.wait_for_service('/small_scout_1/get_true_pose')
		localization_service = rospy.ServiceProxy('/small_scout_1/get_true_pose', srv.LocalizationSrv)
		true_pose_got = None

		try:
			true_pose_got = localization_service(call=True).pose
		except ServiceException:
			rospy.logerr('**********get_true_pose call failed. PLEASE RESTART**************')

		try:
			self.odom_pose = [0, 0, 0]
			self.odom_pose[0] = true_pose_got.position.x
			self.odom_pose[1] = true_pose_got.position.y
			self.odom_pose[2] = true_pose_got.position.z
			# q = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
			q = [true_pose_got.orientation.x, true_pose_got.orientation.y, true_pose_got.orientation.z, true_pose_got.orientation.w]
			h = Rotation.from_quat(q)
			self.heading = (h.as_rotvec())[2]
		except Exception:
			rospy.logerr('Cubesat Detection: Exception in odometry position')
			return


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
		# x_sum = 0.0
		# y_sum = 0.0
		# z_sum = 0.0
		# count = 0.0		

		# for data in pc2.read_points(point_cloud_msg, skip_nans=True):
		# 	x_sum += data[0]
		# 	y_sum += data[1]
		# 	z_sum += data[2]
		# 	count += 1

		# if count < 1:
		# 	if self.debug == True:
		# 		rospy.loginfo('Cubesat Detection: no point cloud detection')
		# 	return
		# else:
		# 	x_avg = x_sum / count
		# 	y_avg = y_sum / count
		# 	z_avg = z_sum / count


		
		data = pc2.read_points_list(point_cloud_msg, skip_nans=True)
		if len(data) < 5:
			return
		# rospy.logwarn(data)
		# x_avg = data[0].x
		# y_avg = data[1].y
		# z_avg = data[2].z

		H = data[5].z # hypotenuse of a right triangle from scout to cubesat (triangle HZV)
		self.distance = H

		# ----------------------------------------------------------------------------------------------------------
		# calculate a transform from the point cloud to our camera frame of reference
		# ONLY THE Z VALUE IS ACCURATE IN THIS TRANSFORM, further processing is needed to get the X and Y using odom
		# ----------------------------------------------------------------------------------------------------------
		try:
			transform = self.tf_buffer.lookup_transform('small_scout_1_tf/base_footprint', point_cloud_msg.header.frame_id, point_cloud_msg.header.stamp, rospy.Duration(2.0))
		except Exception:
			return

		pose_stamped = PoseStamped()
		pose_stamped.header = point_cloud_msg.header
		pose_stamped.pose.position = data[5]
		# pose_stamped.pose.position.y = y_avg
		# pose_stamped.pose.position.z = z_avg
		pose_stamped.pose.orientation = transform.transform.rotation
		try:
			pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
		except Exception:
			return

		
		Z = pose_transformed.pose.position.z # side Z of a right triangle from scout to cubesat (triangle HZV)
		
		# pythagorean thereom: V^2 = H^2 - Z^2; V = sqrt(H^2 - Z^2)
		V = (H * H) - (Z * Z) # side V of a right triangle from scout to cubesat (triangle HZV)
		# consider some edge cases for the calculation of V and make an estimate adjustment
		if (V < 0.0):
			V *= -1.0

		V = math.sqrt(V)

		if H < 25.0 and self.used_truepose == False:
			self.get_truepose()
			self.used_truepose = True
		
		if self.heading != None:
			# V is the magnitude of a vector from the robot to the cubesat, we can use trigonometry to approximate a transform
			# from the XY of the robot to the XY of the cubesat (Z is already calculated above)
			# rospy.loginfo('Heading= ' + str(self.heading))

			self.detection_pose = [0, 0, 0]
			self.detection_pose[2] = Z  + self.odom_pose[2]
			self.detection_pose[1] = (V * math.sin(self.heading - self.heading_correction))  + self.odom_pose[1]
			self.detection_pose[0] = (V * math.cos(self.heading - self.heading_correction))  + self.odom_pose[0]


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


	def cam_callback(self, left_camera_data):
		# convert image data from image message -> opencv image
		try:
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
					if area < 600:
						shape = self.detect(c)
						color = self.label(lab_left,c)
						# rospy.loginfo(color)
						# rospy.loginfo(area)
						if shape == 'rectangle' and color == 'yellow' and color != 'white' and color != 'red':
							c = c.astype("float")
							c *= ratio_left
							c = c.astype("int")
							# rospy.loginfo(area)
							cv2.drawContours(cv_image_left, [c], -1, (0, 255, 0), 3)
							if self.detection_pose != None:
								detection_msg = Detection()
								detection_msg.detection_id = Obstacles.CUBESAT
								detection_msg.x = self.detection_pose[0]
								detection_msg.y = self.detection_pose[1]
								detection_msg.z = self.detection_pose[2]
								detection_msg.distance = self.distance
								detection_msg.heading = ((cX - 320) / 640) * 2.0944 # radians (approx 120 degrees)
								self.heading_correction = detection_msg.heading
								self.cubesat_detection_publisher.publish(detection_msg)



			# publish the detection image topic showing the detected object contour
			# used for debugging and visualisation
			imgmsg_left = self.bridge.cv2_to_imgmsg(cv_image_left, encoding="passthrough")
			self.cubesat_detection_image_left_publisher.publish(imgmsg_left)
		except AttributeError as e:
			rospy.logerr('Cubesat Attribute Error: ' + str(e))
			return

