#!/usr/bin/env python

import rospy
#from std_msgs.msg import String
#from srcp2_msgs import msg
#lastVol = None

def callback(data):
	# code goes here
	return


def play():
    rospy.init_node('disparityMap')
    # rospy.Subscriber("/scout_1/volatile_sensor", msg.VolSensorMsg, callback)
	# need to also create a publisher
    rospy.spin()


if __name__ == '__main__':
    play()
