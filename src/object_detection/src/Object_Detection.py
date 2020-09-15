#!/usr/bin/env python


import rospy
from Volatile_Detection import VolatileDetection
from Logo_Detection import LogoDetection
from Cubesat_Detection import CubesatDetection
from Leg_Detection import LegDetection


if __name__ == '__main__':
	rospy.init_node('scoot_object_detection')
	LogoDetection()
	LegDetection()
	# VolatileDetection()
	CubesatDetection()
	rospy.spin()

