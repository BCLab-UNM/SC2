#! /usr/bin/env python3
import angles
import math
import rospy
import tf
from geometry_msgs.msg import Point


def to_radians(degree):
    return 2 * math.pi * degree / 360.0


def pretty_point(point):
    return "({0:0.2f}, {1:0.2f})".format(point.x, point.y)


class WaypointNavigator:
    def __init__(self, scoot):
        self.scoot = scoot

    def navigate(self, waypoints):
        for i in range(len(waypoints)):
            rospy.loginfo("Going to waypoint {}".format(i))
            self.navigate_to_waypoint(waypoints[i])

    def get_2d_pose(self):
        rotation = self.scoot.OdomLocation.Odometry.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(quaternion=(rotation.x, rotation.y, rotation.z, rotation.w))
        return euler[2]

    def navigate_to_waypoint(self, waypoint):
        distance = float("inf")
        pose_position = Point(0, 0, 0)
        while distance > 0.9:
            pose_position = self.scoot.OdomLocation.Odometry.pose.pose.position
            pose_angle = self.get_2d_pose()
            # TODO: Remove this when the IMU flip is fixed
            pose_angle = -pose_angle
            angle_to = math.atan2(pose_position.y - waypoint.y, -pose_position.x + waypoint.x)
            angle = angles.shortest_angular_distance(pose_angle, angle_to)

            if abs(angle) < to_radians(5):
                # Move forward
                rospy.logdebug("Moving forward")
                self.scoot.drive(1)
            elif abs(angle) < to_radians(70):
                # Soft Turn
                # TODO: Implement with forward movement
                rospy.logdebug("Soft Turn angle: {}".format(angle))
                self.scoot.turn(to_radians(-10 if angle > 0 else 10))
            else:
                # Hard Turn
                rospy.logdebug("Hard Turn angle: {}".format(angle))
                self.scoot.turn(to_radians(-50 if angle > 0 else 50))

            pose_position = self.scoot.OdomLocation.Odometry.pose.pose.position
            distance = math.hypot(waypoint.x - pose_position.x, waypoint.y - pose_position.y)
        rospy.loginfo("Reached waypoint: {} at point: {}".format(pretty_point(waypoint), pretty_point(pose_position)))
