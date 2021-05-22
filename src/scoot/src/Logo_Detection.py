#!/usr/bin/env python3

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


class LogoDetection:
    def __init__(self):
        # note: required library "pip install imutils"

        self.bridge = CvBridge()

        self.point_cloud_subscriber = rospy.Subscriber('/small_scout_1/points2', PointCloud2, self.pc_callback)
        self.scoot_odom_subscriber = rospy.Subscriber('/small_scout_1/odometry/filtered', Odometry, self.odom_callback)
        self.left_camera_subscriber = message_filters.Subscriber('/small_scout_1/camera/left/image_raw', Image)

        self.logo_detection_image_left_publisher = rospy.Publisher('/small_scout_1/logo_detections/image/left', Image,
                                                                   queue_size=100)
        self.logo_detection_left_publisher = rospy.Publisher('/small_scout_1/detections', Detection, queue_size=100)

        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.left_camera_subscriber], 10, 0.1,
                                                                        allow_headerless=True)
        self.synchronizer.registerCallback(self.callback)

        self.z_value_list = []

        # self.colors_blue = OrderedDict()
        # self.colors_blue.update( {"red" : (255,0,0)})
        # self.colors_blue.update( {"blue" : (0,0,255)})

        # for i in range(1, 256):
        # temp = {"blue_" + str(i) : (0,0,i)}
        # self.colors_blue.update(temp)
        colors = OrderedDict({"yellow": (255, 195, 0), "red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255),
                              "white": (255, 255, 255), "black": (0, 0, 0), })

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
        self.left_camera_info_subscriber = message_filters.Subscriber('/small_scout_1/camera/left/camera_info',
                                                                      CameraInfo)
        self.right_camera_info_subscriber = message_filters.Subscriber('/small_scout_1/camera/right/camera_info',
                                                                       CameraInfo)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.left_camera_info_subscriber, self.right_camera_info_subscriber], 10, 0.1, allow_headerless=True)
        self.synchronizer.registerCallback(self.camera_info_callback)
        self.left_camera_focal_length = 380.0

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose_transformed = None
        self.heading = None
        self.heading_correction = None
        self.odom_pose = None

    def odom_callback(self, odom_msg):
        # extract the robot's XYZ position and heading (q) from the odometry message
        try:
            self.odom_pose = [0, 0, 0]
            self.odom_pose[0] = odom_msg.pose.pose.position.x
            self.odom_pose[1] = odom_msg.pose.pose.position.y
            self.odom_pose[2] = odom_msg.pose.pose.position.z
            q = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z,
                 odom_msg.pose.pose.orientation.w]
            h = Rotation.from_quat(q)
            self.heading = (h.as_rotvec())[2]
        except Exception:
            rospy.logerr('Logo Detection: Exception in odometry position')
            return

    def pc_callback(self, point_cloud_msg):
        points_list = []

        for data in pc2.read_points(point_cloud_msg, skip_nans=True):
            points_list.append([data[0], data[1], data[2]])

        if len(points_list) == 0:
            rospy.loginfo('no point cloud')
            return

        # small_scout_1_tf/base_footprint
        try:
            transform = self.tf_buffer.lookup_transform('small_scout_1_tf/base_footprint',
                                                        point_cloud_msg.header.frame_id, rospy.Time(0),
                                                        rospy.Duration(1.0))

            index = int(len(points_list) / 2)
            point = points_list[index]
            pose_stamped = PoseStamped()
            pose_stamped.header = point_cloud_msg.header
            pose_stamped.pose.position.x = point[0]  # see stereo_image_proc docs, the xyz need to be remapped
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

    def calculate_xyz(self, d):
        if self.heading == None:
            return None

        xyz = [0, 0, 0]

        xyz[0] = (d * math.cos(self.heading - self.heading_correction))  # + self.odom_pose[0]
        xyz[1] = (d * math.sin(self.heading - self.heading_correction))  # + self.odom_pose[1]
        xyz[2] = 1.0  # + self.odom_pose[2] # assuming the logo is 1m off of the ground

        return xyz

    def callback(self, left_camera_data):

        try:
            left_detection_msg = Detection()
            left_detection_msg.detection_id = Obstacles.HOME_FIDUCIAL  # this is an integer ID defined in the obstacle package
            left_detection_msg.heading = None
            left_detection_msg.distance = None
            left_detection_msg.x = None
            left_detection_msg.y = None
            left_detection_msg.z = None

            # left_camera_data and right_camera_data are sensor_msg/Image data types

            # convert image data from image message -> opencv image
            cv_image_left = cv2.cvtColor(self.bridge.imgmsg_to_cv2(left_camera_data, desired_encoding="passthrough"),
                                         cv2.COLOR_BGR2RGB)

            # for i in range(240,256):
            # cv_image_left[np.all(cv_image_left == (i,i,i), axis=-1)] = (0,0,0)
            # determine colors

            # generate shapes
            # TODO: re-write for left and right camera
            resized_left = imutils.resize(cv_image_left, width=640)
            ratio_left = resized_left.shape[0] / float(resized_left.shape[0])
            gray_left = cv2.cvtColor(resized_left, cv2.COLOR_BGR2GRAY)
            lab_left = cv2.cvtColor(resized_left, cv2.COLOR_BGR2LAB)
            blurred_left = cv2.GaussianBlur(gray_left, (5, 5), 0)
            thresh_left = cv2.threshold(blurred_left, 110, 255, cv2.THRESH_BINARY)[1]

            # thresh_left = thresh_left.astype(np.uint8)
            cnts_left = cv2.findContours(thresh_left.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_left = imutils.grab_contours(cnts_left)

            for c in cnts_left:
                M = cv2.moments(c)

                if M["m00"] != 0:
                    cX = int((M["m10"] / M["m00"]))
                    cY = int((M["m01"] / M["m00"]))
                    area = cv2.contourArea(c)
                    if area > 200 and area < 700:
                        shape = self.detect(c)
                        color = self.label(lab_left, c)

                        # rospy.loginfo(shape)

                        # if color == 'red':
                        if shape == 'triangle':
                            c = c.astype("float")
                            c *= ratio_left
                            c = c.astype("int")
                            cv2.drawContours(cv_image_left, [c], -1, (0, 255, 0), 3)
                            # rospy.loginfo(color)
                            # rospy.loginfo(area)
                            # print(M['m10'])
                            marker = cv2.minAreaRect(c)
                            focalLength = self.left_camera_focal_length
                            KNOWN_WIDTH = 0.955  # logo width in meterswith
                            per_width = marker[1][0]
                            distance_meters = self.distance_to_camera(KNOWN_WIDTH, focalLength, per_width)

                            self.heading_correction = ((cX - 320) / 640) * 2.0944  # radians (approx 120 degrees)
                            xyz = self.calculate_xyz(distance_meters)

                            if xyz == None:  # we don't have enough data to continue
                                return

                            left_detection_msg.heading = self.heading_correction
                            left_detection_msg.distance = distance_meters
                            left_detection_msg.x = xyz[0]
                            left_detection_msg.y = xyz[1]
                            left_detection_msg.z = xyz[2]

                            self.logo_detection_left_publisher.publish(left_detection_msg)

                # cv2.putText(cv_image_left, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            imgmsg_left = self.bridge.cv2_to_imgmsg(cv_image_left, encoding="passthrough")
            self.logo_detection_image_left_publisher.publish(imgmsg_left)

        except AttributeError as e:
            rospy.logerr('Logo Attribute Error: ' + str(e))
            return
