#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from srcp2_msgs import msg
lastVol = None

def callback(data):
    global lastVol
    jitterDecimal = 1
    if lastVol is None or data.vol_index is not lastVol.vol_index:
        rospy.loginfo("Found new %s, %f from me", data.vol_type, data.distance_to)
    elif round(data.distance_to, jitterDecimal) < round(lastVol.distance_to, jitterDecimal):
        rospy.loginfo("Warmer, %s is %f from me", data.vol_type, data.distance_to)
    elif round(data.distance_to, jitterDecimal) > round(lastVol.distance_to, jitterDecimal):
        rospy.loginfo("Colder, %s is %f from me", data.vol_type, data.distance_to)
    lastVol = data


def play():
    rospy.init_node('warmerColder')
    rospy.Subscriber("/scout_1/volatile_sensor", msg.VolSensorMsg, callback)
    rospy.spin()


if __name__ == '__main__':
    play()
