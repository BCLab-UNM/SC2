#!/usr/bin/env python


import rospy
from volatile_detection import VolatileDetection
from logo_detection import LogoDetection


if __name__ == '__main__':
	rospy.init_node('scoot_object_detection')
	LogoDetection()
	VolatileDetection()
	# other detectors can go here
	rospy.spin()

