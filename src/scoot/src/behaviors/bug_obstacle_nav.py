#!/usr/bin/env python2

# Matthew Fricke, 2020, based on an implementation by Isabelle and Ethan Miller distibuted under the MIT license.

#The three bug algorithms differ only in how they decide to leave the wall and return to the path through free space to the goal. To implement this, a single Bug class was created that contained all of the shared logic of the bug algorithms, with the main loop: while current_location.distance(tx, ty) > delta: hit_wall = bug.go_until_obstacle() if hit_wall: bug.follow_wall() print "Arrived at", (tx, ty)

#where follow_wall() loops until bug.should_leave_wall() is true.

#Bug0 implements logic to see if the path in the direction of the goal is clear. Bug1 implements logic to confirm circumnavigation occured and the robot is at the closest point. Bug2 implements logic to see if the slope of the line to the destination is the same as the slope at impact and the current position is closer.

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
#import roslib; roslib.load_manifest('bugs')
import rospy
import tf.transformations as transform
from geometry_msgs.msg import Twist
from srcp2_msgs import msg, srv
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import threading
import sys

# Location is used to maintain a single current location of the robot in a
# thread-safe manner such that the event handlers and readers can all use it without
# issue
class Location:
    def __init__(self):
        self.m = threading.Lock() # global lock b/c easy and not a problem yet
        self.x = None
        self.y = None
        self.t = None
        self.deltaT = 0.1 # how close to angle to be to go

    def update_location(self, x, y, t):
        self.m.acquire()
        self.x = x
        self.y = y
        self.t = t
        self.m.release()

    def current_location(self):
        self.m.acquire()
        x = self.x
        y = self.y
        t = self.t
        self.m.release()
        return (x, y, t)

    def distance(self, x, y):
        (x0, y0, _) = self.current_location()
        if x0 == None or y0 == None:
            # will be none on the first iteration
            return sys.maxint
        return math.sqrt((x-x0)**2 + (y-y0)**2)

    def facing_point(self, x, y):
        (cx, cy, current_heading) = self.current_location()
        if None in (cx, cy, current_heading):
            return False
        n = necessary_heading(cx, cy, x, y)
        # TODO(exm) possible bug with boundary conditions?
        return n - self.deltaT <= current_heading <= n + self.deltaT

    def faster_left(self, x, y):
        (cx, cy, current_heading) = self.current_location()
        if None in (cx, cy, current_heading):
            return False
        return current_heading - necessary_heading(cx, cy, x, y) < 0

    def global_to_local(self, desired_angle):
        (_, _, current_heading) = self.current_location()
        ans = desired_angle - current_heading
        if ans < -math.pi:
            ans += 2* math.pi
        return ans


# current x, y; target x,y
def necessary_heading(cx, cy, tx, ty):
    return math.atan2(ty-cy, tx-cx)

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
        return int((angle - self.raw.angle_min)/self.raw.angle_increment)

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

current_location = Location()
current_dists = Dist()

delta = 0.1
WALL_PADDING = 1

STRAIGHT = 0
LEFT = 1
RIGHT = 2
BACK = 3
MSG_STOP = 4

def init_listener():
    rospy.init_node('Bug_Obstacle_Nav', anonymous=True)
    rospy.Subscriber('/scout_1/odom/filtered', Odometry, location_handler)
    rospy.Subscriber('/scout_1/laser/scan', LaserScan, lidar_handler)

    print("Waiting for break service...")
    rospy.wait_for_service('/scout_1/brake_rover')
    brakeService = rospy.ServiceProxy('/scout_1/brake_rover', srv.BrakeRoverSrv)
    print("... active.")
    
def location_handler(data):
    p = data.pose.pose.position
    q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
    t = transform.euler_from_quaternion(q)[2] # in [-pi, pi]
    current_location.update_location(p.x, p.y, t)

def lidar_handler(data):
    current_dists.update(data)

class Bug:

    def __init__(self, tx, ty):
        # Robot linear velocity in meters per second
        self.linear_vel = 1.5

        # Robot angular velocity in radians per second
        self.angular_vel = round(math.pi/2,2)
    
        self.pub = rospy.Publisher('/scout_1/skid_cmd_vel', Twist, queue_size=1)
        self.tx = tx
        self.ty = ty

    def stop(self):
        go(MSG_STOP)
        brake_service.call(100)

        
    def go(self, direction):
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = self.linear_vel
            print "Moving forward at ", self.linear_vel, "m/s"
        elif direction == LEFT:
            cmd.angular.z = self.angular_vel
            print "Turning left at ", self.angular_vel, "rad/s"
        elif direction == RIGHT:
            cmd.angular.z = -self.angular_vel
            print "Turning right at ", self.angular_vel, "rad/s"
        elif direction == BACK:
            cmd.linear.x = -self.linear_vel
            print "Backing up at ", self.linear_vel, "m/s"
        elif direction == MSG_STOP:
            pass

        self.pub.publish(cmd)

    # Return True if a wall was encountered otherwise false
    def go_until_obstacle(self):
        print "Going until destination or obstacle."
        print "Current location: ", current_location.current_location()
        print "Distance to target: ", round(current_location.distance(tx, ty)), "m"
        count = 0
        while current_location.distance(tx, ty) > delta:
            (frontdist, _) = current_dists.get()
            if frontdist <= WALL_PADDING:
                self.go(BACK)
                self.print_LiDAR_ranges()
                return True
            
            if current_location.facing_point(tx, ty):
                self.go(STRAIGHT)
            elif current_location.faster_left(tx, ty):
                self.go(LEFT)
            else:
                self.go(RIGHT)
            rospy.sleep(0.1)
            if count % 1 == 0:
                print "Distance to target: ", round(current_location.distance(tx, ty)), "m"
                cx, cy, t = current_location.current_location()
                print "Angle Error: ", necessary_heading(cx, cy, tx, ty)-t, "rad"

                self.print_LiDAR_ranges()
            count += 1
                
        return False

    def print_LiDAR_ranges(self):
        left_range, front_range = current_dists.get()
        if left_range > 100:
            left_range = "max"
        if front_range > 100:
            front_range = "max"
        print "LiDAR range. Front:", round(front_range,2), "m. Left: ", round(left_range,2), "m"
    
    def follow_wall(self):
        print "Following wall"
        while current_dists.get()[0] <= WALL_PADDING:
            self.print_LiDAR_ranges()
            self.go(RIGHT)
            rospy.sleep(0.1)
        while not self.should_leave_wall():
            (front, left) = current_dists.get()
            if front <= WALL_PADDING:
                self.print_LiDAR_ranges()
                self.go(RIGHT)
            elif WALL_PADDING - 0.3 <= left <= WALL_PADDING + .3:
                self.print_LiDAR_ranges()
                self.go(STRAIGHT)
            elif left > WALL_PADDING + 0.3:
                self.go(LEFT)
            else:
                self.go(RIGHT)
            rospy.sleep(0.1)

    def should_leave_wall(self):
        print "You dolt! You need to subclass bug to know how to leave the wall"
        sys.exit(0.1)

class Bug0(Bug):
    def should_leave_wall(self):
        (x, y, t) = current_location.current_location()
        dir_to_go = current_location.global_to_local(necessary_heading(x, y, tx, ty))
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
        (x, y, t) = current_location.current_location()

        if None in self.closest_point:
            self.origin = (x, y)
            self.closest_point = (x, y)
            self.closest_distance = current_location.distance(self.tx, self.ty)
            self.left_origin_point = False
            return False
        d = current_location.distance(self.tx, self.ty)
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
        while not current_location.facing_point(self.tx, self.ty):
            self.go(RIGHT)
            rospy.sleep(.01)

    def follow_wall(self):
        Bug.follow_wall(self)
        self.face_goal()

    def should_leave_wall(self):
        (x, y, _) = current_location.current_location()
        if None in self.encountered_wall_at:
            self.encountered_wall_at = (x, y)
            self.lh = necessary_heading(x, y, self.tx, self.ty)
            return False
        t_angle = necessary_heading(x, y, self.tx, self.ty)
        (ox, oy) = self.encountered_wall_at
        od = math.sqrt((ox-self.tx)**2 + (oy-self.ty)**2)
        cd = math.sqrt( (x-self.tx)**2 +  (y-self.ty)**2)
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
    init_listener()

    print "Waiting for location data on '/scout_1/odom/filtered...'"
    rospy.wait_for_message('/scout_1/odom/filtered', Odometry,)
    print "... received."

    while current_location.distance(tx, ty) > delta:
        hit_wall = bug.go_until_obstacle()
        if hit_wall:
            bug.follow_wall()

    bug.stop()
    print "Arrived at", (tx, ty), "(Final location:", current_location.x, current_location.y, ")"

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

