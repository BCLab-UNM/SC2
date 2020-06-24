#! /usr/bin/env python
import angles, math, tf, rospy

def toRadians(degree):
    return 2 * math.pi * degree / 360.0

def prettyPoint(point):
    return "({0:0.2f}, {1:0.2f})".format(point.x, point.y)

class WaypointNavigator:
    def __init__(self, scoot):
        self.scoot = scoot
        self.current_odom = None

    def position(self, odom):
        self.current_odom = odom

    def getPosition(self):
        return self.current_odom

    def navigate(self, waypoints):
        for i in range(len(waypoints)) :
            rospy.loginfo("Going to waypoint {}".format(i))
            self.navigateToWaypoint(waypoints[i])

    def get2DPose(self):
        rotation = self.current_odom.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(quaternion=(rotation.x, rotation.y, rotation.z, rotation.w))
        return euler[2]


    def navigateToWaypoint(self, waypoint):
        distance = float("inf")
        while distance > 0.9 :
            posePosition = self.current_odom.pose.pose.position
            poseAngle = self.get2DPose()
            # TODO: Remove this when the IMU flip is fixed
            poseAngle = -poseAngle
            angleto = math.atan2(posePosition.y - waypoint.y, -posePosition.x + waypoint.x)
            angle = angles.shortest_angular_distance(poseAngle, angleto)

            if abs(angle) < toRadians(5):
                # Move forward
                rospy.logdebug("Moving forward")
                self.scoot.drive(1)
            elif abs(angle) < toRadians(70):
                # Soft Turn
                # TODO: Implement with forward movement
                rospy.logdebug("Soft Turn angle: {}".format(angle))
                self.scoot.turn(toRadians(-10 if angle > 0 else 10))
            else:
                # Hard Turn
                rospy.logdebug("Hard Turn angle: {}".format(angle))
                self.scoot.turn(toRadians(-50 if angle > 0 else 50))

            posePosition = self.current_odom.pose.pose.position
            distance = math.hypot(waypoint.x - posePosition.x, waypoint.y - posePosition.y)
        rospy.loginfo("Reached waypoint: {} at point: {}".format(prettyPoint(waypoint), prettyPoint(posePosition)))