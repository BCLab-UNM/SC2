#!/usr/bin/env python3

# Generates odometry from wheel poses 

import math 
import sys
import tf.transformations as transform
from sensor_msgs.msg import JointState, Imu
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Converts wheel poses into wheel odometry
class WheelEncoder:

    def __init__(self):
        self.name = rospy.get_param('rover_name', default='small_scout_1')

        print("Subscribing to /{}/imu".format(self.name))
        print("Subscribing to /{}/joint_states".format(self.name))
        print("Publishing to /{}/odom".format(self.name)) 
        
        rospy.Subscriber("/{}/imu".format(self.name), Imu, self.imuCallback)
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
        #self.wheel_radius = 0.27 # meters
        self.wheel_radius = 0.17 # meters
        # self.track_width = 1.75 # meters
        self.track_width = 1 # meters
        self.sample_rate = 1
        self.msg_min_delta = 0.1 # seconds <--- Changing this from 0.1 spoils the accuracy. That should not be the case!
        self.message_count = 0

    def imuCallback(self, data):
        q = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
        roll, pitch, yaw = transform.euler_from_quaternion(q) # in [-pi, pi]
        self.theta = yaw + 0.78 # rotate 45 degrees 

        
    def jointStatesCallback(self, data):

        # Only process the message at the desired sample rate
        self.message_count += 1

        if self.message_count % self.sample_rate != 0:
            return

        # Divide by 180 to get radians
        back_left_wheel_angle = data.position[ data.name.index('bl_wheel_joint') ]*math.pi/180
        back_right_wheel_angle = data.position[ data.name.index('br_wheel_joint') ]*math.pi/180
        front_left_wheel_angle = data.position[ data.name.index('fl_wheel_joint') ]*math.pi/180
        front_right_wheel_angle = data.position[ data.name.index('fr_wheel_joint') ]*math.pi/180
        
        current_time = rospy.Time.now()  
        dt = (current_time - self.last_time).to_sec()

        # Check that sufficient time has passed to avoid duplicate time stamps
        if dt < self.msg_min_delta: # Specify the minimum time between messages for function execution (to prevent timestamp warning killing the repl)
            return
        
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

        print("Wheel Radius: ", self.wheel_radius)

        v_rx = ( v_right + v_left ) / 2.0
        v_ry = 0

        print("Velocity: ", v_rx, " m/s") 
        print("Yaw: ", self.theta)
        
        # We increase the track width by a factor of 4 to match empirical tests
        v_rtheta = ( v_right - v_left ) / 2*self.track_width
        
        # Velocities and pose in the odom frame
        # The velocities expressed in the robot base frame can be transformed into the odom frame. 
        # To transform velocities into the odom frame we only need to rotate them according to the current robot orientation. 
        # We are operating in the x-y plane, we ignore altitude - which will result in errors as the robot climbs up and down.
        # Integrate the velocities over time (multiply by the time differential dt and sum)

        #v_wx = (v_rx * cos(self.theta) - v_ry * sin(self.theta)) * dt
        #v_wy = (v_rx * sin(self.theta) + v_ry * cos(self.theta)) * dt

        # We know our heading from the imu - no need to calculate it from wheel velocities
        v_wx = v_rx*cos(self.theta)
        v_wy = v_rx*sin(self.theta)

        print("X displacement: ", v_wx, " m")
        print("Y displacement: ", v_wy, " m")
        print("Delta_t: ", dt, " s")

        print("Back Left Wheel Angle: ", back_left_wheel_angle)
        
        #v_wtheta = v_rtheta * dt 
                
        self.x += v_wx*2*math.pi # I don't understand why this 2pi factor is needed
        self.y += v_wy*2*math.pi

        print("X coord: ", self.x)
        print("Y coord: ", self.y)

        # Replaced with IMU theta
        #self.theta += v_wtheta
        
        # Composing your odometry message
        
        # The odometry message contains pose and twist information. The pose represents your current robot pose in the the odom frame (x, y, theta).
                
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        # Check if the timestamp will be different
        
        
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

    print("Wheel encoder node started")
    
    # Register shutdown handler (includes ctrl-c handling)
    rospy.on_shutdown( shutdownHandler )

    # Initialise the node
    encoder = WheelEncoder()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
