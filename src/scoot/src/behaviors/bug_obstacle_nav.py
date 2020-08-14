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

# Location is used to maintain a single current location of the robot in a
# thread-safe manner such that the event handlers and readers can all use it without
# issue

class Dist:
    def __init__(self):
        self.m = threading.Lock()
        self.left = 0
        self.front = 0
        self.raw = []

    def update(self, data):
        def getmin(a, b):
            in_rng = lambda x: data.range_min <= x <= data.range_max
            vsp = filter(in_rng, data.ranges[a:b])
            if len(vsp) > 0:
                return min(vsp)
            else:
                return sys.maxint

        newfront = getmin(82, 120)
        newleft = getmin(70, 81)

        self.m.acquire()
        self.left = newleft
        self.front = newfront
        self.raw = data
        self.m.release()

    def get(self):
        self.m.acquire()
        l = self.left
        f = self.front
        self.m.release()
        return (f, l)

    def angle_to_index(self, angle):
        return int((angle - self.raw.angle_min) / self.raw.angle_increment)

    # angle in radians
    def at(self, angle):
        def getmin(a, b):
            in_rng = lambda x: self.raw.range_min <= x <= self.raw.range_max
            vsp = filter(in_rng, self.raw.ranges[a:b])
            if len(vsp) > 0:
                return min(vsp)
            else:
                return sys.maxint

        self.m.acquire()
        i = self.angle_to_index(angle)
        start = i - 40
        if start < 0:
            start = 0
        end = i + 40
        if end >= len(self.raw.ranges):
            end = len(self.raw.ranges) - 1
        ans = getmin(start, end)
        self.m.release()
        return ans


class Bug:

    MSG_STOP = 4
    def __init__(self, tx, ty):
        self.delta = 0.1
        self.WALL_PADDING = 1

        self.STRAIGHT = 0
        self.LEFT = 1
        self.RIGHT = 2
        self.BACK = 3

        # Robot linear velocity in meters per second
        self.linear_dist = 1.5

        # Robot angular velocity in radians per second
        self.angle = round(math.pi / 2, 2)
        self.tx = tx
        self.ty = ty

    def stop(self):
        global scoot
        scoot.brake()

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
            pass

    # Return True if a wall was encountered otherwise false
    def go_until_obstacle(self):
        print "Going until destination or obstacle."
        print "Current location: ", scoot.getOdomLocation()
        print "Distance to target: ", round(scoot.OdomLocation.distance(self.tx, self.ty)), "m"
        count = 0
        while scoot.OdomLocation.distance(self.tx, self.ty) > self.delta:
            (frontdist, _) = self.current_dists.get()
            if frontdist <= self.WALL_PADDING:
                self.go(self.BACK)
                self.print_LiDAR_ranges()
                return True

            if scoot.OdomLocation.facing_point(self.tx, self.ty):
                self.go(self.STRAIGHT)
            elif scoot.OdomLocation.faster_left(self.tx, self.ty):
                self.go(self.LEFT)
            else:
                self.go(self.RIGHT)
            rospy.sleep(0.1)
            if count % 1 == 0:
                print "Distance to target: ", round(scoot.OdomLocation.distance(self.tx, self.ty)), "m"
                cx, cy, t = scoot.OdomLocation.current_location()
                print "Angle Error: ", necessary_heading(cx, cy, self.tx, self.ty) - t, "rad"

                self.print_LiDAR_ranges()
            count += 1

        return False

    def print_LiDAR_ranges(self):
        left_range, front_range = current_dists.get()
        if left_range > 100:
            left_range = "max"
        if front_range > 100:
            front_range = "max"
        print "LiDAR range. Front:", round(front_range, 2), "m. Left: ", round(left_range, 2), "m"

    def follow_wall(self):
        print "Following wall"
        while current_dists.get()[0] <= self.WALL_PADDING:
            self.print_LiDAR_ranges()
            self.go(self.RIGHT)
            rospy.sleep(0.1)
        while not self.should_leave_wall():
            (front, left) = current_dists.get()
            if front <= self.WALL_PADDING:
                self.print_LiDAR_ranges()
                self.go(self.RIGHT)
            elif self.WALL_PADDING - 0.3 <= left <= self.WALL_PADDING + .3:
                self.print_LiDAR_ranges()
                self.go(self.STRAIGHT)
            elif left > self.WALL_PADDING + 0.3:
                self.go(self.LEFT)
            else:
                self.go(self.RIGHT)
            rospy.sleep(0.1)

    def should_leave_wall(self):
        print "You dolt! You need to subclass bug to know how to leave the wall"
        sys.exit(0.1)


class Bug0(Bug):
    def should_leave_wall(self):
        (x, y, t) = scoot.OdomLocation.current_location()
        dir_to_go = scoot.OdomLocation.global_to_local(necessary_heading(x, y, self.tx, self.ty))
        at = current_dists.at(dir_to_go)
        if at > 10:
            print "Leaving wall"
            return True
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


def lidar_handler(data):
    current_dists.update(data)


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

    current_dists = Dist()

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
