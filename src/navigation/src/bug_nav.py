#!/usr/bin/env python2

# Matthew Fricke, 2020, based on an implementation by Isabelle and Ethan Miller distibuted under the MIT license.

#The three bug algorithms differ only in how they decide to leave the wall and return to the path through free space to the goal. To implement this, a single Bug class was created that contained all of the shared logic of the bug algorithms, with the main loop: while current_location.distance(tx, ty) > delta: hit_wall = bug.go_until_obstacle() if hit_wall: bug.follow_wall() rospy.logwarn( "Arrived at", (tx, ty)

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
import rospy
import tf.transformations as transform
from geometry_msgs.msg import Twist, Point
from srcp2_msgs import msg, srv
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from obstacle.msg import Obstacles
import threading
import sys
import random
from signal import signal, SIGINT
from tqdm import tqdm # For progress bars

import Queue

# Robot name prefix for topics
robot_name = None

# Constants
OBSTACLE_EDGE_TOL = 1 # meters
max_num_waypoints = 0
waypoint_bounds = 75 # meters. distance from center to edges
waypoint_queue = Queue.Queue( max_num_waypoints )
# Limit on reaching waypoint
waypoint_timeout = None
timed_out = False
start_time = 0

delta = 2 # meters. How close the robot tries to get to a waypoint
WALL_PADDING = 3 # meters. This has to be carefully set to balance not running into objects and thinking slopes are obstacles

# Track success stats
success_count = 0.0
success_distance = 0.0
success_time = 0.0
stats_printed = False
total_time_start = 0
status_msg = None

escape_waypoint = None
STRAIGHT = 0
LEFT = 1
RIGHT = 2
BACK = 3
MSG_STOP = 4
CLOCKWISE = 5
ANTICLOCKWISE = 6

current_location = None
current_dists = None

# Timout exception
class TimedOutException(Exception):
    pass

def random_waypoint_generator( n_waypoints ):
    pub = rospy.Publisher('/'+robot_name+'/waypoints', Point, queue_size=1)
    rospy.logwarn("Generating Waypoints...")
    for i in tqdm(range(n_waypoints)):
        wp = Point(random.uniform(-waypoint_bounds, waypoint_bounds), random.uniform(-40, 40), 0)
        pub.publish(wp)
        rospy.sleep(0.1)
    rospy.logwarn("Finished")

# Message Handlers

# Processes move to waypoint requests. Waypoints are given as geometry points
def waypoint_handler( msg ):
    
    rospy.logwarn( "[BugNav] New waypoint recieved: Coords <"+ str(msg.x) + ","+ str(msg.y) + ","+ str( msg.z ) +">" )
    rospy.logwarn( "[BugNav] Waypoint Queue Length:" + str( waypoint_queue.qsize() ) )

    global max_num_waypoints
    max_num_waypoints += 1
    
    waypoint_queue.put(msg)
    
# Location is used to maintain a single current location of the robot in a
# thread-safe manner such that the event handlers and readers can all use it without
# issue
class Location:
    def __init__(self):
        self.m = threading.Lock() # global lock b/c easy and not a problem yet
        self.x = None
        self.y = None
        self.t = None
        self.pitch = 0.0
        self.deltaT = 0.25 # how close to angle to be to go

    def update_location(self, x, y, t, pitch):
        self.m.acquire()
        self.x = x
        self.y = y
        self.t = t
        self.pitch = pitch
        self.m.release()

    def current_location(self):
        self.m.acquire()
        x = self.x
        y = self.y
        t = self.t
        pitch = self.pitch
        self.m.release()
        return (x, y, t, pitch)

    def distance(self, x, y):
        (x0, y0, _, _) = self.current_location()
        if x0 == None or y0 == None:
            # will be none on the first iteration
            return sys.maxint
        return math.sqrt((x-x0)**2 + (y-y0)**2)

    def facing_point(self, x, y):
        (cx, cy, current_heading, _) = self.current_location()
        if None in (cx, cy, current_heading):
            return False
        n = necessary_heading(cx, cy, x, y)
        # TODO(exm) possible bug with boundary conditions?
        return n - self.deltaT <= current_heading <= n + self.deltaT

    def faster_left(self, x, y):
        (cx, cy, current_heading, _) = self.current_location()
        if None in (cx, cy, current_heading):
            return False
        return current_heading - necessary_heading(cx, cy, x, y) < 0

    def global_to_local(self, desired_angle):
        (_, _, current_heading, _) = self.current_location()
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
        self.right = 0

    def update(self, data):
        def getmin(a, b):
            in_rng = lambda x: data.range_min <= x <= data.range_max
            vsp = filter(in_rng, data.ranges[a:b])
            if len(vsp) > 0:
                return min(vsp)
            else:
                return sys.maxint

        newfront = getmin(40, 60)
        newleft = getmin(60, 100)
        newright = getmin(0, 40)

        self.m.acquire()
        self.left = newleft
        self.front = newfront
        self.right = newright
        self.raw = data
        self.m.release()
        
    def get(self):
        self.m.acquire()
        l = self.left
        f = self.front
        r = self.right
        self.m.release()
        return (f, l, r)

    def angle_to_index(self, angle):
        return int((angle - self.raw.angle_min)/self.raw.angle_increment)

    # angle in radians
    #def at(self, angle):
    #    def getmin(a, b):
    #        in_rng = lambda x: self.raw.range_min <= x <= self.raw.range_max
    #        vsp = filter(in_rng, self.raw.ranges[a:b])
    #        if len(vsp) > 0:
    #            return min(vsp)
    #        else:
    #            return sys.maxint
    #    self.m.acquire()
    #    i = self.angle_to_index(angle)
    #    start = i - 40
    #    if start < 0:
    #        start = 0
    #    end = i + 40
    #    if end >= len(self.raw.ranges):
    #        end = len(self.raw.ranges) - 1
    #    ans = getmin(start, end)
    #    self.m.release()
    #    return ans

    def at( self ):
        #self.m.acquire()
        #index_min = min(range(len(self.raw.ranges)), key=self.raw.ranges.__getitem__)
        #self.m.release()

        return min(self.raw.ranges)
    

def init_listener():
    rospy.Subscriber('/'+robot_name+'/odometry/filtered', Odometry, estimated_location_handler)
    rospy.Subscriber('/'+robot_name+'/odom/filtered', Odometry, actual_location_handler)
    rospy.Subscriber('/'+robot_name+'/laser/scan', LaserScan, lidar_handler)
    rospy.Subscriber("/scout_1/imu", Imu, imu_handler)
    rospy.Subscriber('/'+robot_name+'/obstacle', Obstacles, obstacle_handler) 
    
    rospy.logwarn("[BugNav] Waiting for brake service...")
    rospy.wait_for_service('/'+robot_name+'/brake_rover')
    brakeService = rospy.ServiceProxy('/'+robot_name+'/brake_rover', srv.BrakeRoverSrv)
    rospy.logwarn("[BugNav] ... active.")

    waypoint_topic = "/"+robot_name+"/waypoints"
    rospy.Subscriber('/'+robot_name+'/waypoints', Point, waypoint_handler)
    rospy.logwarn("[BugNav] Subscribing to "+ waypoint_topic)

    rospy.logwarn( "[BugNav] Waiting for location data on /"+robot_name+"/odom/filtered...")
    rospy.wait_for_message("/"+robot_name+"/odom/filtered", Odometry)
    rospy.logwarn( "[BugNav] ... received.")

    rospy.logwarn( "[BugNav] Waiting for location data on /"+robot_name+"/odometry/filtered...")
    rospy.wait_for_message("/"+robot_name+"/odometry/filtered", Odometry,)
    rospy.logwarn( "[BugNav] ... received.")

    rospy.logwarn("[BugNav] Waiting for break service...")
    rospy.wait_for_service("/"+robot_name+"/brake_rover")
    global brake_service
    brake_service = rospy.ServiceProxy('/'+robot_name+'/brake_rover', srv.BrakeRoverSrv)
    rospy.logwarn("... active.")
    
def estimated_location_handler(data):
    p = data.pose.pose.position
    q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
    roll, pitch, yaw = transform.euler_from_quaternion(q) # in [-pi, pi]
    estimated_current_location.update_location(p.x, p.y, yaw, pitch)

def actual_location_handler(data):
    p = data.pose.pose.position
    q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
    roll, pitch, yaw = transform.euler_from_quaternion(q) # in [-pi, pi]
    actual_current_location.update_location(p.x, p.y, yaw, pitch)

def lidar_handler(data):
    current_dists.update(data)

def imu_handler( data ):
    q = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
    roll, pitch, yaw = transform.euler_from_quaternion(q) # in [-pi, pi]

def obstacle_handler(data):
    pass
    
class Bug:

    def __init__(self):
        # Robot linear velocity in meters per second
        self.linear_vel = rospy.get_param("linear_velocity", default=5)

        # Robot angular velocity in radians per second
        self.angular_vel = rospy.get_param("angular_velocity", default=5)
    
        self.pub = rospy.Publisher('/'+robot_name+'/skid_cmd_vel', Twist, queue_size=1)
        self.tx = None
        self.ty = None

        self.stuck_linear_tol = 2
        self.stuck_angular_tol = math.pi/4

        # We only want one function driving at a time
        self.drive_mutex = threading.Lock()
        
        # Remember where we were before
        self.last_x = sys.maxint
        self.last_y = sys.maxint
        self.last_h = sys.maxint

        # How long to check between struck checks
        self.stuck_check_period = 20

        
        # Setup a timer to check if we are stuck
        self.stuck_timer = rospy.Timer(rospy.Duration(self.stuck_check_period), self.stuck_handler)

        
    def apply_brakes(self):
        
        brake_service.call(100)
        # rospy.logwarn( "Applied Brakes"

    def release_brakes(self):
        brake_service.call(0)
        #rospy.logwarn( "Released Brakes"
        
    def stuck_handler(self, event=None):
        
        # Check if we are stuck
        #rospy.logwarn( "#########################"
        #rospy.logwarn( "#  Stuck handler called #"
        #rospy.logwarn( "#########################"
        #self.rospy.logwarn(_error()

        # Check for timeout
        elapsed_time = rospy.get_rostime().secs - start_time
        #rospy.logwarn( waypoint_timeout - elapsed_time
        if  elapsed_time > waypoint_timeout:
            global timed_out
            timed_out = True
            return
        
        x, y, h, pitch = estimated_current_location.current_location()
        
        #rospy.logwarn( "delta_x: ", abs(x - self.last_x)
        #rospy.logwarn( "delta_y: ", abs(y - self.last_y)
        #rospy.logwarn( "delta_h: ", abs(h - self.last_h)
    
        if estimated_current_location.distance(self.last_x, self.last_y) < self.stuck_linear_tol and estimated_current_location.distance(self.tx, self.ty) > delta:
            self.drive_mutex.acquire()
            #rospy.logwarn( "Escaping: Robot displaced by", current_location.distance(self.last_x, self.last_y), "meters over", self.stuck_check_period, " seconds."
            cmd = Twist()
            cmd.linear.x = self.linear_vel*random.randint(-1,1)
            if cmd.linear.x == 0:
                cmd.angular.z = self.angular_vel
                #rospy.logwarn( "Escape: turning at ", cmd.angular.z, "rad/s"
            else:
                pass
                #rospy.logwarn( "Escape: driving at ", cmd.linear.x, "m/s"
            for i in range(10):
                self.pub.publish(cmd)
            rospy.sleep(3)
            self.drive_mutex.release()
            
        
        #global escape_waypoint
        #if abs(x - self.last_x) < self.stuck_linear_tol and abs(y - self.last_y) < self.stuck_linear_tol and abs(h - self.last_h) < self.stuck_angular_tol and current_location.distance(self.tx, self.ty) > delta:
            #wp = Point(random.uniform(-waypoint_bounds, waypoint_bounds), random.uniform( -waypoint_bounds,  waypoint_bounds), 0)
        #    wp = Point(0, 0, 0)
        #    escape_waypoint = wp
        #    rospy.logwarn( "Setting escape waypoint:", (wp.x, wp.y)
        #    waypoint_queue.put(wp)
        #else:
        #    if escape_waypoint != None:
        #        if escape_waypoint != waypoint_queue.queue[0]:
        #            rospy.logwarn( "Escaped: WARNING! The escape waypoint was not at the head of the queue! Not removing."
        #        else:
        #            waypoint_queue.get()
        #            rospy.logwarn( "Escaped: removing escape waypoint from queue"
        #        escape_waypoint = None
        
            
        self.last_x = x
        self.last_y = y
        self.last_h = h

        self.stuck = False # We hope, if not this function will be exectuted again
        
    def go(self, direction):

        # Check for timeout
        if timed_out:
            raise TimedOutException()
        
        # Do nothing if someone else is driving (avoids blocking mutex lock)
        if self.drive_mutex.locked():
            #rospy.logwarn( "go(): Someone else is driving"
            pass

        #self.rospy.logwarn(_LiDAR_ranges()

        # Add noise so we don't get into loops
        linear_vel = self.linear_vel + random.gauss(0, 1)

        # Robot angular velocity in radians per second
        angular_vel = self.angular_vel + random.gauss(0, 1)

        command_reps = 10
        
        self.drive_mutex.acquire()
        self.release_brakes()
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = linear_vel
            #rospy.logwarn( "Moving forward at ", self.linear_vel, "m/s"
        elif direction == LEFT:
           # cmd.linear.x = self.linear_vel/10
            cmd.angular.z = angular_vel
            #rospy.logwarn( "Turning left at ", self.angular_vel, "rad/s"
        elif direction == RIGHT:
            #cmd.linear.x = -self.linear_vel/10
            cmd.angular.z = -angular_vel
            #rospy.logwarn( "Turning right at ", self.angular_vel, "rad/s"
        elif direction == BACK:
            cmd.linear.x = -linear_vel
            #rospy.logwarn( "Backing up at ", self.linear_vel, "m/s"
        elif direction == MSG_STOP:
            #rospy.logwarn( "Stopping"
            cmd.angular.z = 0
            cmd.linear.x = 0
            self.apply_brakes()


        for i in range(command_reps):
            self.pub.publish(cmd)
        rospy.sleep(0.1)
        self.drive_mutex.release()
        
    def print_error(self):
        cx, cy, t, pitch = estimated_current_location.current_location()
        rospy.logwarn( "[BugNav] Estamated distance to target: " + str(round(estimated_current_location.distance(self.tx, self.ty))) + "m" )
        rospy.logwarn( "[BugNav] Actual distance to target: " + str(round(actual_current_location.distance(self.tx, self.ty))) + "m" )
        rospy.logwarn( "[BugNav] Angle Error: " + str(necessary_heading(cx, cy, self.tx, self.ty)-t) + " "+ "rad" )
        
    # Return True if a wall was encountered otherwise false
    def go_until_obstacle(self):
        #rospy.logwarn( "[BugNav] Going until destination or obstacle."
        #print_error()
        #rospy.logwarn( "[BugNav] Travelling to waypoint"
        
        while estimated_current_location.distance(self.tx, self.ty) > delta:
            (frontdist, leftdist, rightdist) = current_dists.get()
            _, _, _, pitch = estimated_current_location.current_location()
            
            if frontdist <= WALL_PADDING and leftdist <= WALL_PADDING:
                #self.go(MSG_STOP)
                self.go(BACK)
                #print_LiDAR_ranges()
                return ANTICLOCKWISE
            elif frontdist <= WALL_PADDING and rightdist <= WALL_PADDING:
                #self.go(MSG_STOP)
                self.go(BACK)
                #print_LiDAR_ranges()
                return CLOCKWISE
            elif frontdist <= WALL_PADDING:
                self.go(BACK)
                return CLOCKWISE
            #elif rightdist <= WALL_PADDING:
            #    self.go(LEFT)
            #elif leftdist <= WALL_PADDING:
            #    self.go(RIGHT)
            
            if estimated_current_location.facing_point(self.tx, self.ty):
                self.go(STRAIGHT)
            elif estimated_current_location.faster_left(self.tx, self.ty):
                self.go(LEFT)
            else:
                self.go(RIGHT)
                        
        return False

    def print_LiDAR_ranges(self):
        front_range, left_range, right_range = current_dists.get()
        if left_range > 100:
            left_range = "max"
        if front_range > 100:
            front_range = "max"
        if right_range > 100:
            right_range = "max"
       
        rospy.logwarn( "[BugNav] LiDAR range. Front:" + " " + front_range+ " " + "m. Left: "+ " " + left_range+ " " + "m. Right: "+ " " + right_range+ " " + "m" )
    
    def follow_wall_anticlockwise(self):
        #rospy.logwarn( "[BugNav] Navigating around obstacle anticlockwise"
        while current_dists.get()[0] <= WALL_PADDING:
            #print_LiDAR_ranges()
            #rospy.logwarn( "[BugNav] Aligning with obstacle"
            self.go(RIGHT)
            rospy.sleep(0.1)
        while not self.should_leave_wall() and estimated_current_location.distance(self.tx, self.ty) > delta:
            rospy.sleep(0.1)
            (front, left, right) = current_dists.get()
            #if front <= WALL_PADDING-OBSTACLE_EDGE_TOL:
            #    self.go(BACK)
            #elif front <= WALL_PADDING:
            if front <= WALL_PADDING:
                #print_LiDAR_ranges()
                #rospy.logwarn( "[BugNav] Still aligning with obstacle"
                self.go(RIGHT)
            elif WALL_PADDING - OBSTACLE_EDGE_TOL <= left <= WALL_PADDING + OBSTACLE_EDGE_TOL:
                #rospy.logwarn( "[BugNav] Following obstacle edge"
                #print_LiDAR_ranges()
                self.go(STRAIGHT)
            elif left > WALL_PADDING + 0.5:
                #rospy.logwarn( "[BugNav] Getting too far away from obstacle"
                self.go(LEFT)
            elif front > WALL_PADDING and left > WALL_PADDING and right > WALL_PADDING:
                self.go(STRAIGHT)
                #rospy.logwarn( "[BugNav] Free of obstacle"
                return
            else:
                #rospy.logwarn( "[BugNav] Aligning with obstacle again."
                self.go(RIGHT)
            
        # print_error()
        # rospy.logwarn( "[BugNav] Left Obstacle"

    def follow_wall_clockwise(self):
        #rospy.logwarn( "[BugNav] Navigating around obstacle clockwise"
        while current_dists.get()[0] <= WALL_PADDING:
            #print_LiDAR_ranges()
            #rospy.logwarn( "[BugNav] Aligning with obstacle"
            self.go(LEFT)
            rospy.sleep(0.1)
        while not self.should_leave_wall() and estimated_current_location.distance(self.tx, self.ty) > delta:
            rospy.sleep(0.1)
            (front, left, right) = current_dists.get()
            #if front <= WALL_PADDING-OBSTACLE_EDGE_TOL:
            #    self.go(BACK)
            #elif front <= WALL_PADDING:
            if front <= WALL_PADDING:
                #print_LiDAR_ranges()
                #rospy.logwarn( "[BugNav] Still aligning with obstacle"
                self.go(LEFT)
            elif WALL_PADDING - OBSTACLE_EDGE_TOL <= right <= WALL_PADDING + OBSTACLE_EDGE_TOL:
                #rospy.logwarn( "[BugNav] Following obstacle edge"
                #print_LiDAR_ranges()
                self.go(STRAIGHT)
            elif left > WALL_PADDING + 0.5:
                #rospy.logwarn( "[BugNav] Getting too far away from obstacle"
                self.go(RIGHT)
            elif front > WALL_PADDING and left > WALL_PADDING and right > WALL_PADDING:
                self.go(STRAIGHT)
                #rospy.logwarn( "[BugNav] Free of obstacle"
                return
            else:
                #rospy.logwarn( "[BugNav] Aligning with obstacle again."
                self.go(LEFT)
            
        # print_error()
        # rospy.logwarn( "[BugNav] Left Obstacle"

    def should_leave_wall(self):
        rospy.logwarn( "[BugNav] You dolt! You need to subclass bug to know how to leave the wall")
        sys.exit(0.1)

class Bug0(Bug):

    # If we are pointing towards the target location and the way is clear leave the obstacle
    def should_leave_wall(self):
        (x, y, t, _) = estimated_current_location.current_location()
        dir_to_go = estimated_current_location.global_to_local(necessary_heading(x, y, self.tx, self.ty))

        if abs(dir_to_go - t) < math.pi/4 and current_dists.get()[0] > 5:
                #print_error()
                # rospy.logwarn( "[BugNav] Leaving obstacle"
                return True
        return False

class Bug1(Bug):
    def __init__(self, tx, ty):
        Bug.__init__(self, tx, ty)
        self.closest_point = (None, None)
        self.origin = (None, None)
        self.circumnavigated = False

    def should_leave_wall(self):
        (x, y, t, _) = estimated_current_location.current_location()

        if None in self.closest_point:
            self.origin = (x, y)
            self.closest_point = (x, y)
            self.closest_distance = estimated_current_location.distance(self.tx, self.ty)
            self.left_origin_point = False
            return False
        d = estimated_current_location.distance(self.tx, self.ty)
        if d < self.closest_distance:
            rospy.logwarn( "[BugNav] New closest point at"+ " " + (x, y) )
            self.closest_distance = d
            self.closest_point = (x, y)

        (ox, oy) = self.origin
        if not self.left_origin_point and not near(x, y, ox, oy):
            # we have now left the point where we hit the wall
            rospy.logwarn( "[BugNav] Left original touch point" )
            self.left_origin_point = True
        elif near(x, y, ox, oy) and self.left_origin_point:
            # circumnavigation achieved!
            rospy.logwarn( "[BugNav] Circumnavigated obstacle" )
            self.circumnavigated = True

        (cx, ct) = self.closest_point
        if self.circumnavigated and near(x, y, cx, ct):
            self.closest_point = (None, None)
            self.origin = (None, None)
            self.circumnavigated = False
            self.left_origin_point = False
            rospy.logwarn( "[BugNav] Leaving wall")
            return True

        else:
            return False

class Bug2(Bug):
    def __init__(self, tx, ty):
        Bug.__init__(self, tx, ty)
        self.lh = None
        self.encountered_wall_at = (None, None)

    def face_goal(self):
        while not estimated_current_location.facing_point(self.tx, self.ty):
            self.go(RIGHT)
            rospy.sleep(.01)

    def follow_wall(self):
        Bug.follow_wall(self)
        self.face_goal()

    def should_leave_wall(self):
        (x, y, _, _) = estimated_current_location.current_location()
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
                rospy.logwarn( "[BugNav] Leaving wall" )
                return True
        return False

def near(cx, cy, x, y):
    nearx = x - .3 <= cx <= x + .3
    neary = y - .3 <= cy <= y + .3
    return nearx and neary

def bug_algorithm(bug_type=0):

    # Track success stats
    global success_count
    global success_distance
    global success_time
    global stats_printed
    global total_time_start
   
    if bug_type == 0:
         bug = Bug0()
    elif bug_type == 1:
        bug = Bug1()
    elif bug_type == 2:
        bug = Bug2()
    else:
        rospy.logwarn( "[BugNav] Unknown Bug algorithm" + " " +  bug_type)
        sys.exit(3)
        
    # For status messages so other nodes know when we are done or if we failed
    status_topic = "/"+robot_name+"/bug_nav_status"
    bug_nav_status_publisher = rospy.Publisher(status_topic, String, queue_size=10)
    rospy.logwarn( "[BugNav] Publishing status messages on" + status_topic )

    # Add the command line waypoint to the queue
    # waypoint_queue.put(Point(tx, ty, 0))

    # Generate waypoints - use a thread so we don't continue until the waypoints are completed
    #thread = threading.Thread(target=random_waypoint_generator( max_num_waypoints ))
    #thread.start()

    # wait here for waitpoint generation to complete
    #thread.join()
    

    # Track total time spent
    total_time_start = rospy.get_rostime().secs
    
    
    # Check for new waypoints every 10 seconds
    idle = rospy.Rate(2)
    
    ###### main waypoint consumer loop  - run till node shuts down  ######
    while not rospy.is_shutdown():
        idle.sleep()


        rospy.logwarn( "[BugNav] Waiting for waypoints..." )
        rospy.wait_for_message("/"+robot_name+"/waypoints", Point)
        
        # Process waypoint queue, or if there are none and we are not at the coords provided on the
        # command line go there.
        while not waypoint_queue.empty():
            
            waypoint = waypoint_queue.get()
            wtx = waypoint.x
            wty = waypoint.y

            bug.tx = wtx
            bug.ty = wty

            # Begin timout timer
            global start_time
            start_time = rospy.get_rostime().secs
            est_distance_to_cover = estimated_current_location.distance(wtx, wty)
            act_distance_to_cover = actual_current_location.distance(wtx, wty)
            rospy.logwarn( "[BugNav] Est (x,y):" + " " + "(" + str(estimated_current_location.current_location()[0]) + "," + str(estimated_current_location.current_location()[1]) )
            rospy.logwarn( "[BugNav] Actual (x,y):"+ " " +  "(" + str(actual_current_location.current_location()[0]) + ", " + str(actual_current_location.current_location()[1]) )
            rospy.logwarn( "[BugNav] Moving to coordinates from waypoint:" + " " + "(" + str(round(wtx,2)) + ", " + str(round(wty,2)) + " " + "Distance: "+ " " + str(round(est_distance_to_cover,2)) + " " + "m." )
            rospy.logwarn( "[BugNav] Actual Distance: "+ " " + str(round(act_distance_to_cover,2)) + " " + "m." )
            global status_msg
            while estimated_current_location.distance(wtx, wty) > delta:
                try:
                    # These two functions are the heart of the algorithm. "Go_until_obstacle" moves towards the target location when there are no
                    # detected obstacles.
                    # The second (wall_follow) navigates around obstacles and positions the rover so that it can move towards the
                    # target location again
                    circumnavigate_obstacle = bug.go_until_obstacle()
                    if circumnavigate_obstacle == CLOCKWISE:
                        bug.follow_wall_clockwise()
                    elif circumnavigate_obstacle == ANTICLOCKWISE:
                        bug.follow_wall_anticlockwise()
                except TimedOutException:
                    elapsed_time = rospy.get_rostime().secs - start_time
                    rospy.logwarn( "[BugNav] Failed to reach"+ " (" +  str(round(wtx,2)) + ", " + str(round(wty,2)) + ") after"+ " " + str(round(elapsed_time)) + " " + "(sim) seconds. Distance: "+ " " + str(round(estimated_current_location.distance(wtx, wty),2)) )
                    status_msg = "Timeout:", (wtx, wty)
                    bug_nav_status_publisher.publish(status_msg)

                    global timed_out
                    timed_out = False
                    break
                    
            # Confirm the target location was reached
            if estimated_current_location.distance(wtx, wty) < delta:

                bug.go(MSG_STOP)
                bug.apply_brakes()
                elapsed_time = rospy.get_rostime().secs - start_time
                rospy.logwarn( "[BugNav] Arrived at" + " " + "(" + str(round(wtx,2)) + ", " + str(round(wty,2)) + " ) " + " after "+ str(round(elapsed_time)) + " seconds. Distance: " + " " + str(round(actual_current_location.distance(wtx, wty),2)))
                status_msg = "Arrived!"
                bug_nav_status_publisher.publish(status_msg)
                if escape_waypoint == None:
                    success_count += 1.0
                    success_distance += act_distance_to_cover
                    success_time += elapsed_time 
                
            rospy.logwarn(  "[BugNav] There are"+ " " + str(waypoint_queue.qsize())+ " " + "waypoints remaining." )

        if not stats_printed:
            try:
                success_perc = round((success_count/max_num_waypoints)*100)
            except ZeroDivisionError:
                success_perc = 0.0
            rospy.logwarn( "[BugNav] Succeeded: " + " " + str(success_perc) + " " + "% of the time." )
            rospy.logwarn( "[BugNav] Distance covered: "+ " " + str(round(success_distance,2))+ " " + "m" )
            rospy.logwarn( "[BugNav] Time spent on successful runs: "+ " " + str(round(success_time, 2)) + " " + "s" )
            try:
                avg_speed = round(success_distance/success_time,2)
            except ZeroDivisionError:
                avg_speed = 0.0
            rospy.logwarn( "[BugNav] Avg Speed: "+ " " + str(avg_speed) + " " + "m/s" )
            # Track total time spent
            total_time_elapsed = rospy.get_rostime().secs - total_time_start
            rospy.logwarn( "[BugNav] Total Time: "+ " " + str(round(total_time_elapsed,2)) + " " + "s" )
    
            
            stats_printed = True
            
        idle.sleep()
       

def sigint_handler(signal_received, frame):
    waypoints_processed = max_num_waypoints-waypoint_queue.qsize()
    rospy.logwarn( "[BugNav] Processed"+ " " + str(waypoints_processed) + " " +"waypoints." )
    try:
        success_perc = round((success_count/waypoints_processed)*100)
    except ZeroDivisionError:
        success_perc = 0.0
    rospy.logwarn( "[BugNav] Succeeded: "+ " " + str(success_perc) + " " + "% of the time." )
    rospy.logwarn( "[BugNav] Distance covered: "+ " " + str(round(success_distance,2)) + " " + "m" )
    rospy.logwarn( "[BugNav] Time spent on successful runs: "+ " " + str(round(success_time,2)) + " " + "s" )
    try:
        avg_speed = round(success_distance/success_time,2)
    except ZeroDivisionError:
        avg_speed = 0.0
    rospy.logwarn( "[BugNav] Avg Speed: "+ " " + str(avg_speed) + " " + "m/s" )
    # Track total time spent
    total_time_elapsed = rospy.get_rostime().secs - total_time_start
    rospy.logwarn( "[BugNav] Total Time: "+ " " + str(round(total_time_elapsed,2)) + " " + "s" ) 
    
    rospy.logwarn( "[BugNav] SIGINT or CTRL-C received. Exiting." )
    exit(0)

def main( task=None ):
    global estimated_current_location
    global actual_current_location
    global current_dists
    global status_msg
    global waypoint_timeout
    waypoint_timeout = rospy.get_param("waypoint_timeout", default=300) # Seconds

    
    global robot_name
    robot_name = rospy.get_param('rover_name', default='scout_1')
    
    estimated_current_location = Location()
    actual_current_location = Location()
    current_dists = Dist()

    init_listener()
    signal(SIGINT, sigint_handler)

    bug_algorithm()
                
if __name__ == '__main__':
    rospy.init_node('Bug_Obstacle_Nav', anonymous=True, log_level=rospy.DEBUG)
    rospy.logwarn('Bug nav started.')
    
    sys.exit(main())

