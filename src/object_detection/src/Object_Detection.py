#!/usr/bin/env python


import rospy
from Volatile_Detection import VolatileDetection
from Logo_Detection import LogoDetection
from Cubesat_Detection import CubesatDetection


if __name__ == '__main__':
	rospy.init_node('scoot_object_detection')
	# LogoDetection()
	# VolatileDetection()
	CubesatDetection()
	rospy.spin()

