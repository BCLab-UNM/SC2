#!/usr/bin/env python


import rospy
from Volatile_Detection import VolatileDetection
from Logo_Detection import LogoDetection


if __name__ == '__main__':
	rospy.init_node('scoot_object_detection')
	LogoDetection()
	VolatileDetection()
	# other detectors can go here
	rospy.spin()

