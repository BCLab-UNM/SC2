#!/usr/bin/env python3
from gazebo_msgs.srv import GetModelState
import rospy
import math

def main():
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    cur_pose = model_coordinates("scout_1", 'world').pose.position
    all_vols_poses =[ model_coordinates("vol_"+str(num), "world").pose.position for num in range(0,29) ]
    closest_vol_pose = min(all_vols_poses, key=lambda k: math.sqrt((k.x - cur_pose.x) ** 2 + (k.y - cur_pose.y) ** 2))
    print("Rover pose:")
    print(cur_pose)
    print("\n\n Closest Volatile pose:")
    print(closest_vol_pose)


if __name__ == '__main__':
    main()
