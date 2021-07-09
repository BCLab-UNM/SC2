from __future__ import print_function

import sys

import rospy
import math

import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

M_PI_2 = 1.5708
M_PI = M_PI_2 * 2

class DriveController:

    def __init__(self):
        self.rover_namespace = rospy.get_namespace()
        self.rover_name = self.rover_namespace.strip('/')
        
        rospy.Subscriber('/' + self.rover_name + '/cmd_vel', Twist, self._drive)
    
        self.back_left_wheel_drive = rospy.Publisher('/' + self.rover_name + '/back_left_wheel/drive/command/velocity', Float64, queue_size=10)
        self.back_right_wheel_drive = rospy.Publisher('/' + self.rover_name + '/back_right_wheel/drive/command/velocity', Float64, queue_size=10)
        self.front_left_wheel_drive = rospy.Publisher('/' + self.rover_name + '/front_left_wheel/drive/command/velocity', Float64, queue_size=10)
        self.front_right_wheel_drive = rospy.Publisher('/' + self.rover_name + '/front_right_wheel/drive/command/velocity', Float64, queue_size=10)

        self.back_left_wheel_steer = rospy.Publisher('/' + self.rover_name + '/back_left_wheel/steer/command/position', Float64, queue_size=10)
        self.back_right_wheel_steer = rospy.Publisher('/' + self.rover_name + '/back_right_wheel/steer/command/position', Float64, queue_size=10)
        self.front_left_wheel_steer = rospy.Publisher('/' + self.rover_name + '/front_left_wheel/steer/command/position', Float64, queue_size=10)
        self.front_right_wheel_steer = rospy.Publisher('/' + self.rover_name + '/front_right_wheel/steer/command/position', Float64, queue_size=10)

        self.wheel_radius = 0.17
        self.wheel_base = 1.0
        self.steering_track = 1.0

    
    def clip_steering_angle(self, steer, vel):
        max_steer = M_PI_2
        min_steer = -1 * M_PI_2
        if (steer > max_steer) and (steer - M_PI > min_steer):
            return steer - M_PI, -1 * vel
        if (steer < min_steer) and (steer + M_PI < max_steer):
            return steer + M_PI, -1 * vel
        return steer, vel

    def _drive(self, msg):
        lin_x = msg.linear.x
        lin_y = msg.linear.y
        ang_z = msg.angular.z

        y_comp_minus_ang = lin_y - ang_z * self.wheel_base / 2.0
        y_comp_plus_ang  = lin_y + ang_z * self.wheel_base / 2.0
        x_comp_minus_ang = lin_x - ang_z * self.steering_track / 2.0
        x_comp_plus_ang  = lin_x + ang_z * self.steering_track / 2.0

        vel_left_front  = math.hypot(y_comp_plus_ang, x_comp_minus_ang) / self.wheel_radius
        vel_right_front = math.hypot(y_comp_plus_ang, x_comp_plus_ang) / self.wheel_radius
        vel_left_back   = math.hypot(y_comp_minus_ang, x_comp_minus_ang) / self.wheel_radius
        vel_right_back  = math.hypot(y_comp_minus_ang, x_comp_plus_ang) / self.wheel_radius

        front_left_steering = math.atan2(y_comp_plus_ang, x_comp_minus_ang)
        front_right_steering = math.atan2(y_comp_plus_ang, x_comp_plus_ang)
        back_left_steering = math.atan2(y_comp_minus_ang, x_comp_minus_ang)
        back_right_steering = math.atan2(y_comp_minus_ang, x_comp_plus_ang)

        front_left_steering, vel_left_front = self.clip_steering_angle(front_left_steering, vel_left_front)
        front_right_steering, vel_right_front = self.clip_steering_angle(front_right_steering, vel_right_front)
        back_left_steering, vel_left_back = self.clip_steering_angle(back_left_steering, vel_left_back)
        back_right_steering, vel_right_back = self.clip_steering_angle(back_right_steering, vel_right_back)
                
        self.__drive_front_left_wheel(vel_left_front)
        self.__drive_front_right_wheel(vel_right_front)
        self.__drive_back_left_wheel(vel_left_back)
        self.__drive_back_right_wheel(vel_right_back)

        self.__steer_front_left_wheel(front_left_steering)
        self.__steer_front_right_wheel(front_right_steering)
        self.__steer_back_left_wheel(back_left_steering)
        self.__steer_back_right_wheel(back_right_steering)
  
    def __drive_front_left_wheel(self, velocity):
        v = Float64()
        v.data = velocity
        self.front_left_wheel_drive.publish(v)

    def __drive_front_right_wheel(self, velocity):
        v = Float64()
        v.data = velocity
        self.front_right_wheel_drive.publish(v)

    def __drive_back_left_wheel(self, velocity):
        v = Float64()
        v.data = velocity
        self.back_left_wheel_drive.publish(v)

    def __drive_back_right_wheel(self, velocity):
        v = Float64()
        v.data = velocity
        self.back_right_wheel_drive.publish(v)

    def __steer_front_left_wheel(self, angle):
        v = Float64()
        v.data = angle
        self.front_left_wheel_steer.publish(v)

    def __steer_front_right_wheel(self, angle):
        v = Float64()
        v.data = angle
        self.front_right_wheel_steer.publish(v)

    def __steer_back_left_wheel(self, angle):
        v = Float64()
        v.data = angle
        self.back_left_wheel_steer.publish(v)

    def __steer_back_right_wheel(self, angle):
        v = Float64()
        v.data = angle
        self.back_right_wheel_steer.publish(v)

def shutdownHandler():
    print("Drive controller shutting down.")

if __name__ == '__main__':
    
    rospy.init_node('drive_controller', anonymous=True)
    print('init_node')
    rospy.on_shutdown( shutdownHandler )
    controller = DriveController()

    while not rospy.is_shutdown():
        rospy.spin()
 



