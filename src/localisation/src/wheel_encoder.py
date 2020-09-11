#!/usr/bin/env python

# Generates odometry from wheel poses 

import sys
from sensor_msgs.msg import JointState
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Converts wheel poses into wheel odometry
class WheelEncoder:



    def __init__(self):
        self.name = rospy.get_param('rover_name', default='scout_1')

        rospy.Subscriber("/{}/joint_states".format(self.name), JointState, self.jointStatesCallback)
        self.odom_pub = rospy.Publisher("/{}/odom".format(self.name), Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.last_time = rospy.Time.now()  
        self.x = 0
        self.y = 0
        self.theta = 0
        self.previous_front_left_wheel_angle = 0
        self.previous_front_right_wheel_angle = 0
        self.previous_back_left_wheel_angle = 0
        self.previous_back_right_wheel_angle = 0
        self.wheel_radius = 0.27 # meters
        self.track_width = 1.75 # meters
        
    def jointStatesCallback(self, data):
                
        back_left_wheel_angle = data.position[ data.name.index('bl_wheel_joint') ]
        back_right_wheel_angle = data.position[ data.name.index('br_wheel_joint') ]
        front_left_wheel_angle = data.position[ data.name.index('fl_wheel_joint') ]
        front_right_wheel_angle = data.position[ data.name.index('fr_wheel_joint') ]
        
        current_time = rospy.Time.now()  
        dt = (current_time - self.last_time).to_sec()
                
        ### Calculate angular velocity ###
        
        # Angular displacement in radians - could use all four wheels, start with just two
        angle_displacement_left_front = front_left_wheel_angle - self.previous_front_left_wheel_angle 
        angle_displacement_right_front = front_right_wheel_angle - self.previous_front_right_wheel_angle

        angle_displacement_left_back = back_left_wheel_angle - self.previous_back_left_wheel_angle 
        angle_displacement_right_back = back_right_wheel_angle - self.previous_back_right_wheel_angle 

        angle_displacement_left = (angle_displacement_left_front + angle_displacement_left_back)/2.0
        angle_displacement_right = (angle_displacement_right_front + angle_displacement_right_back)/2.0
        
        # Left angular velocities in rad/s
        try:
            omega_left = angle_displacement_left/dt
            omega_right = angle_displacement_right/dt
        except ZeroDivisionError:
            omega_left = omega_right = 0
            
        # Velocities at the center of the base_footprint, i.e. in the robot frame
        # Subscript r indicates varables in the robot frame
        # v_left:  left velocity
        # v_right: right velocity
        # track_width: the distance between left and right wheels (track)
        # v_rx:    velocity at the center of the robot
        # v_ry:    holonomic constraint (zero for non-holonomic)
        # v_rtheta: rotational velocity

        # Linear velocities in m/s
        v_left = omega_left * self.wheel_radius
        v_right = omega_right * self.wheel_radius

        v_rx = ( v_right + v_left ) /2
        v_ry = 0
        # We increase the track width by a factor of 4 to match empirical tests
        v_rtheta = ( v_right - v_left ) / (4 * self.track_width)
        
        # Velocities and pose in the odom frame
        # The velocities expressed in the robot base frame can be transformed into the odom frame. 
        # To transform velocities into the odom frame we only need to rotate them according to the current robot orientation. 
        # We are operating in the x-y plane, we ignore altitude - which will result in errors as the robot climbs up and down.
        # Integrate the velocities over time (multiply by the time differential dt and sum)

        v_wx = (v_rx * cos(self.theta) - v_ry * sin(self.theta)) * dt
        v_wy = (v_rx * sin(self.theta) + v_ry * cos(self.theta)) * dt
        v_wtheta = v_rtheta * dt 
                
        self.x += v_wx
        self.y += v_wy
        self.theta += v_wtheta
        
        # Composing your odometry message

        # The odometry message contains pose and twist information. The pose represents your current robot pose in the the odom frame (x, y, theta).
                
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            current_time,
            "{}_tf/base_footprint".format(self.name),
            "odom"
        )

        # And link up the map
        self.odom_broadcaster.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            current_time,
            "odom",
            "map"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "{}_tf/base_footprint".format(self.name)
        odom.twist.twist = Twist(Vector3(v_rx, v_ry, 0), Vector3(0, 0, v_rtheta))

        # publish the message
        self.odom_pub.publish(odom)

        # Remember the time so we can calculate dt next time we are called
        self.last_time = current_time

        # Save values for velocity calculations
        self.previous_front_left_wheel_angle = front_left_wheel_angle                                
        self.previous_front_right_wheel_angle = front_right_wheel_angle
        self.previous_back_left_wheel_angle = back_left_wheel_angle
        self.previous_back_right_wheel_angle = back_right_wheel_angle                                
        
def shutdownHandler():
    print("Wheel encoder shutting down.")

if __name__ == '__main__':
    
    rospy.init_node('wheel_encoder', anonymous=True)

    # Register shutdown handler (includes ctrl-c handling)
    rospy.on_shutdown( shutdownHandler )

    # Initialise the node
    encoder = WheelEncoder()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
