#!/usr/bin/env python2

# Matthew Fricke, 2020, based on an implementation by Isabelle and Ethan Miller distibuted under the MIT license.

# The three bug algorithms differ only in how they decide to leave the wall and return to the path through free space to the goal. To implement this, a single Bug class was created that contained all of the shared logic of the bug algorithms, with the main loop: while current_location.distance(tx, ty) > delta: hit_wall = bug.go_until_obstacle() if hit_wall: bug.follow_wall() print "Arrived at", (tx, ty)

# where follow_wall() loops until bug.should_leave_wall() is true.

# Bug0 implements logic to see if the path in the direction of the goal is clear. Bug1 implements logic to confirm circumnavigation occured and the robot is at the closest point. Bug2 implements logic to see if the slope of the line to the destination is the same as the slope at impact and the current position is closer.

# Code for navigating around obstacles as opposed to simply avoiding them.
# Implements bug 0 algorithm
# Assumes that the destination coordinates are provided and that all obstacles
# are convex.

# Bug 0
# 1) Head towards target coordinates
# 2) Follow obstacles (random initial turn) until can head towards goal again
# 3) Release control

# Bug 1
# 1) Head towards target coordinates
# 2) When obstacle encountered circumnavigate and remember the minimum distance
#    between the robot and the target coordinates
# 3) Return to that closest point by wall following and release control

# Bug 2
# Description TODO

import math
import sys
# import roslib; roslib.load_manifest('bugs')
import rospy
import tf.transformations as transform
import threading
import sys
from Scoot import Scoot, ObstacleException
from obstacle.msg import Obstacles

# The bug class defines how to circumnavigate an obstacle
# The only difference between Bug 0, 1, and 2 is how sophisticated
# they are in deciding when to stop circumnavigating
class Bug:

    MSG_STOP = 4
    def __init__(self, tx, ty):
        self.delta = 0.1

        # Encode directions
        self.STRAIGHT = 0
        self.LEFT = 1
        self.RIGHT = 2
        self.BACK = 3

        # Robot movement step in m
        self.linear_dist = 1.5

        # Robot angular step in radians
        self.angle = round(math.pi / 8, 2)

        # Set the target coordinates
        self.tx = tx
        self.ty = ty

    # Stop using the break command
    def stop(self):
        global scoot
        scoot.brake()

    # Function to turn movement commands into scoot function calls
    def go(self, direction):
        global scoot
        try:
            if direction == self.STRAIGHT:
                print "Moving forward at ", self.linear_dist, "m"
                scoot.drive(self.linear_dist, ignore=Obstacles.IS_VISION | Obstacles.IS_VOLATILE)
            elif direction == self.LEFT:
                print "Turning left at ", self.angle, "rads"
                scoot.turn(self.angle, ignore=Obstacles.IS_VISION | Obstacles.IS_VOLATILE)
            elif direction == self.RIGHT:
                print "Turning right at ", self.angle, "rads"
                scoot.turn(-1 * self.angle, ignore=Obstacles.IS_VISION | Obstacles.IS_VOLATILE)
            elif direction == self.BACK:
                print "Backing up at ", self.linear_dist, "m"
                scoot.drive(-1 * self.linear_dist, ignore=Obstacles.IS_VISION | Obstacles.IS_VOLATILE)
            elif direction == self.MSG_STOP:
                scoot.brake()
        except ObstacleException:
            return False

        return True

    # Move towards the goal coordinates until an obstacle gets in the way.
    # Return True if a wall was encountered otherwise false
    def go_until_obstacle(self):
        print "Going until destination or obstacle."
        print "Current location: ", scoot.getOdomLocation()
        print "Distance to target: ", round(scoot.OdomLocation.distance(self.tx, self.ty)), "m"

        try: 
            while scoot.OdomLocation.distance(self.tx, self.ty) > self.delta:
                if scoot.OdomLocation.facing_point(self.tx, self.ty):
                    self.go(self.STRAIGHT)
                elif scoot.OdomLocation.faster_left(self.tx, self.ty):
                    self.go(self.LEFT)
                else:
                    self.go(self.RIGHT)
                rospy.sleep(0.1)
        except ObstacleException:
            print "Encountered obstacle"
            return True

        print "Reached target coordinates without encountering an obstacle."
        return False

    # Follow the "wall" until the appropriate Bug "should leave wall" check is true (the Bug subclass determines the criteria for "should leave wall")
    def follow_wall(self):
        print "Circumnavigating obstacle"

        # Encountering obstacle. First turn to the right into the obstacle is more than wall padding distance ahead (i.e. we turned somewhat parallel to the obstacle)
        try:
            while not rospy.is_shutdown():
                self.go(self.STRAIGHT)
        except DriveException as e:
            # Just the front
            if e == Obstacles.LIDAR_FRONT:
                self.go(self.RIGHT)

        # While shouldn't stop going round the obstacle yet... (i.e. haven't hit the departure point)
        # Turn right if there is an obstacle in front
        # Go straight if there is an obstacle on the left (i.e. following the edge of the obstacle)
        # If we are getting too far away from the obstacle (range on the left is greater than the wall padding + a little leeway)
        # ... then turn left towards the obstacle so we get closer
        # Otherwise we are too close to the obstacle and should turn away.

        
        while not rospy.is_shutdown():
            try:
                print "Don't see obstacle on left or ahead. Turning left towards the obstacle and back towards the target coordinates"
                self.go(self.LEFT)

                # Check to see if we should return to moving towards the target or keep circumnavigating
                if self.should_leave_wall():
                    print "Criteria for leaving the obstacle were met. Exiting wall following function."
                    return 
            except DriveException as e:
                if e == Obstacles.LIDAR_CENTER:
                    print "Too close to obstacle. Turning right."
                    self.go(self.RIGHT)
                elif e == Obstacles.LIDAR_LEFT:
                    print "Still have obstacle on left. Going straight"
                    self.go(self.STRAIGHT)
                elif e == Obstacles.LIDAR_RIGHT:
                    print "Rock and hard place. Backing up and turning right to try and follow the perimeter."
                    self.go(self.BACK)
                    self.go(self.RIGHT)
                
    
    def should_leave_wall(self):
        print "Subclass bug to define when to leave the wall"
        sys.exit(1)


class Bug0(Bug):
    # Try to leave the wall if we are pointing towards the target coordinates
    # Return false if an obstacle exception is triggered when we try to move
    # straight towards the target coords. Return true if the way is clear.
    def should_leave_wall(self):
        if scoot.OdomLocation.facing_point(self.tx, self.ty):
            try:
                self.go(self.STRAIGHT)
                return True
            except DriveException as e:
                if e == Obstacles.LIDAR_CENTER:
                    return False

class Bug1(Bug):
    def __init__(self, tx, ty):
        Bug.__init__(self, tx, ty)
        self.closest_point = (None, None)
        self.origin = (None, None)
        self.circumnavigated = False

    def should_leave_wall(self):

        (x, y, t) = scoot.OdomLocation.current_location()

        if None in self.closest_point:
            self.origin = (x, y)
            self.closest_point = (x, y)
            self.closest_distance = scoot.OdomLocation.distance(self.tx, self.ty)
            self.left_origin_point = False
            return False
        d = scoot.OdomLocation.distance(self.tx, self.ty)
        if d < self.closest_distance:
            print "New closest point at", (x, y)
            self.closest_distance = d
            self.closest_point = (x, y)

        (ox, oy) = self.origin
        if not self.left_origin_point and not near(x, y, ox, oy):
            # we have now left the point where we hit the wall
            print "Left original touch point"
            self.left_origin_point = True
        elif near(x, y, ox, oy) and self.left_origin_point:
            # circumnavigation achieved!
            print "Circumnavigated obstacle"
            self.circumnavigated = True

        (cx, ct) = self.closest_point
        if self.circumnavigated and near(x, y, cx, ct):
            self.closest_point = (None, None)
            self.origin = (None, None)
            self.circumnavigated = False
            self.left_origin_point = False
            print "Leaving wall"
            return True

        else:
            return False


class Bug2(Bug):
    def __init__(self, tx, ty):
        Bug.__init__(self, tx, ty)
        self.lh = None
        self.encountered_wall_at = (None, None)

    def face_goal(self):
        while not scoot.OdomLocation.facing_point(self.tx, self.ty):
            self.go(self.RIGHT)
            rospy.sleep(.01)

    def follow_wall(self):
        Bug.follow_wall(self)
        self.face_goal()

    def should_leave_wall(self):
        (x, y, _) = scoot.OdomLocation.current_location()
        if None in self.encountered_wall_at:
            self.encountered_wall_at = (x, y)
            self.lh = necessary_heading(x, y, self.tx, self.ty)
            return False
        t_angle = necessary_heading(x, y, self.tx, self.ty)
        (ox, oy) = self.encountered_wall_at
        od = math.sqrt((ox - self.tx) ** 2 + (oy - self.ty) ** 2)
        cd = math.sqrt((x - self.tx) ** 2 + (y - self.ty) ** 2)
        dt = 0.01

        if self.lh - dt <= t_angle <= self.lh + dt and not near(x, y, ox, oy):
            if cd < od:
                print "Leaving wall"
                return True
        return False


def near(cx, cy, x, y):
    nearx = x - .3 <= cx <= x + .3
    neary = y - .3 <= cy <= y + .3
    return nearx and neary


def bug_algorithm(bug):
    global scoot
    while scoot.OdomLocation.distance(self.tx, self.ty) > self.delta:
        hit_wall = bug.go_until_obstacle()
        if hit_wall:
            bug.follow_wall()

    bug.stop()
    print "Arrived at", (self.tx, self.ty), "(Final location:", scoot.getOdomLocation.x, scoot.getOdomLocation.y, ")"

def location_handler(data):
    p = data.pose.pose.position
    q = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    t = transform.euler_from_quaternion(q)[2]  # in [-pi, pi]
    scoot.OdomLocation.update_location(p.x, p.y, t)

# current x, y; target x,y
def necessary_heading(cx, cy, tx, ty):
    return math.atan2(ty - cy, tx - cx)


def main(task=None):
    global scoot
    if task:
        scoot = task.scoot
    else:  # Called without task instance
        scoot = Scoot("scout_1")
        scoot.start(node_name='bug_obstacle_nav')
    rospy.loginfo('Bug obstacle nav Started')

    # Parse arguments
    algorithm = sys.argv[1]
    algorithms = ["bug0", "bug1", "bug2"]
    if algorithm not in algorithms:
        print "First argument should be one of ", algorithms, ". Was ", algorithm
        sys.exit(1)

    if len(sys.argv) < 4:
        print "Usage: rosrun scoot bug_obstacle_nav.py bug<0|1|2> target_x target_y"
        sys.exit(1)
    (tx, ty) = map(float, sys.argv[2:4])

    print "Setting target:", (tx, ty)
    bug = None
    if algorithm == "bug0":
        bug = Bug0(tx, ty)
    elif algorithm == "bug1":
        bug = Bug1(tx, ty)
    elif algorithm == "bug2":
        bug = Bug2(tx, ty)

    bug_algorithm(bug)

    sys.exit(0)  # "succeeded"


if __name__ == '__main__':
    rospy.init_node('bug_obstacle_nav')
    sys.exit(main())
