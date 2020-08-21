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
import rospy
import tf.transformations as transform
from geometry_msgs.msg import Twist, Point
from srcp2_msgs import msg, srv
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import threading
import sys
import random
from signal import signal, SIGINT

import Queue

# Timout exception
class TimedOutException(Exception):
    pass

max_num_waypoints = 3
waypoint_queue = Queue.Queue( max_num_waypoints )


def random_waypoint_generator( n_waypoints ):
    pub = rospy.Publisher('/scout_1/waypoints', Point, queue_size=1)

    for i in range(n_waypoints):
        wp = Point(random.uniform(-40, 40), random.uniform(-40, 40), 0)
        pub.publish(wp)
        rospy.sleep(0.1)


# Message Handlers

# Processes move to waypoint requests. Waypoints are given as geometry points
def waypoint_handler( msg ):
    
    # print("New waypoint recieved: Coords <", msg.x, ",", msg.y, ",", msg.z,">")
    # print("Waypoint Queue Length:", waypoint_queue.qsize())
    
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
        self.deltaT = 0.25 # how close to angle to be to go

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

        newfront = getmin(45, 55)
        newleft = getmin(80, 100)

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
    
current_location = Location()
current_dists = Dist()

# Limit on reaching waypoint
waypoint_timeout = 300
timed_out = False
start_time = 0

delta = 2
WALL_PADDING = 3

STRAIGHT = 0
LEFT = 1
RIGHT = 2
BACK = 3
MSG_STOP = 4

def init_listener():
    rospy.init_node('Bug_Obstacle_Nav', anonymous=True)
    rospy.Subscriber('/scout_1/odom/filtered', Odometry, location_handler)
    rospy.Subscriber('/scout_1/laser/scan', LaserScan, lidar_handler)

    # print("Waiting for brake service...")
    rospy.wait_for_service('/scout_1/brake_rover')
    brakeService = rospy.ServiceProxy('/scout_1/brake_rover', srv.BrakeRoverSrv)
    # print("... active.")

    waypoint_topic = "/scout_1/waypoints"
    # print("Subscribing to", waypoint_topic)
    rospy.Subscriber('/scout_1/waypoints', Point, waypoint_handler)

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
        self.linear_vel = 9

        # Robot angular velocity in radians per second
        self.angular_vel = round(2*math.pi,2)
    
        self.pub = rospy.Publisher('/scout_1/skid_cmd_vel', Twist, queue_size=1)
        self.tx = tx
        self.ty = ty

        self.stuck_linear_tol = 0.1
        self.stuck_angular_tol = math.pi/4

        # We only want one function driving at a time
        self.drive_mutex = threading.Lock()
        
        # Remember where we were before
        self.last_x = sys.maxint
        self.last_y = sys.maxint
        self.last_h = sys.maxint

        # How long to check between struck checks
        self.stuck_check_period = 5

        
        # Setup a timer to check if we are stuck
        self.stuck_timer = rospy.Timer(rospy.Duration(self.stuck_check_period), self.stuck_handler)

        
    def apply_brakes(self):
        brake_service.call(100)
        #print "Applied Brakes"

    def release_brakes(self):
        brake_service.call(0)
        #print "Released Brakes"
        
    def stuck_handler(self, event=None):
        # Check if we are stuck
        #print "#########################"
        #print "#  Stuck handler called #"
        #print "#########################"
        #self.print_error()

        # Check for timeout
        elapsed_time = rospy.get_rostime().secs - start_time
        #print waypoint_timeout - elapsed_time
        if  elapsed_time > waypoint_timeout:
            global timed_out
            timed_out = True
            return
        
        x, y, h = current_location.current_location()
        
        #print "delta_x: ", abs(x - self.last_x)
        #print "delta_y: ", abs(y - self.last_y)
        #print "delta_h: ", abs(h - self.last_h)
            
        cmd = Twist()
        
        if abs(x - self.last_x) < self.stuck_linear_tol and abs(y - self.last_y) < self.stuck_linear_tol and abs(h - self.last_h) < self.stuck_angular_tol and current_location.distance(self.tx, self.ty) > delta:
            # print "Looks like we are stuck. Trying to escape."
            self.drive_mutex.acquire()
            self.release_brakes()
            
            #print "Driving at: ", cmd.linear.x, " Angle: ", cmd.angular.z

            #for i in range(10):
            cmd.linear.x = -self.linear_vel
            self.pub.publish(cmd)
            rospy.sleep(4)
            cmd.angular.z = 0
            cmd.linear.x = 0
            self.pub.publish(cmd)
            rospy.sleep(4)
            self.apply_brakes()
            self.release_brakes()
            cmd.angular.z = random.uniform(math.pi, 2*math.pi)
            self.pub.publish(cmd)
            rospy.sleep(4)
            cmd.linear.x = self.linear_vel
            self.pub.publish(cmd)
            rospy.sleep(4)
            self.apply_brakes()
            self.release_brakes()
            self.drive_mutex.release()
        else:
            # print "Not stuck."
            pass
        
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
            return
        
        self.drive_mutex.acquire()
        self.release_brakes()
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = self.linear_vel
            #print "Moving forward at ", self.linear_vel, "m/s"
        elif direction == LEFT:
           # cmd.linear.x = self.linear_vel/10
            cmd.angular.z = self.angular_vel
            #print "Turning left at ", self.angular_vel, "rad/s"
        elif direction == RIGHT:
            #cmd.linear.x = -self.linear_vel/10
            cmd.angular.z = -self.angular_vel
            #print "Turning right at ", self.angular_vel, "rad/s"
        elif direction == BACK:
            cmd.linear.x = -self.linear_vel
            #print "Backing up at ", self.linear_vel, "m/s"
        elif direction == MSG_STOP:
            #print "Stopping"
            cmd.angular.z = 0
            cmd.linear.x = 0
            self.apply_brakes()

        self.pub.publish(cmd)
        rospy.sleep(0.1)
        self.drive_mutex.release()
        
    def print_error(self):
        print "Distance to target: ", round(current_location.distance(self.tx, self.ty)), "m"
        cx, cy, t = current_location.current_location()
        print "Angle Error: ", necessary_heading(cx, cy, self.tx, self.ty)-t, "rad"
        
    # Return True if a wall was encountered otherwise false
    def go_until_obstacle(self):
        #print "Going until destination or obstacle."
        #self.print_error()
        while current_location.distance(self.tx, self.ty) > delta:
            (frontdist, _) = current_dists.get()
            if frontdist <= WALL_PADDING:
                self.go(BACK)
                #self.print_LiDAR_ranges()
                return True
            
            if current_location.facing_point(self.tx, self.ty):
                self.go(STRAIGHT)
            elif current_location.faster_left(self.tx, self.ty):
                self.go(LEFT)
            else:
                self.go(RIGHT)
            rospy.sleep(0.1)
                        
        return False

    def print_LiDAR_ranges(self):
        front_range, left_range = current_dists.get()
        if left_range > 100:
            left_range = "max"
        if front_range > 100:
            front_range = "max"
        print "LiDAR range. Front:", front_range, "m. Left: ", left_range, "m"
    
    def follow_wall(self):
        # print "Navigating around obstacle"
        while current_dists.get()[0] <= WALL_PADDING:
            #self.print_LiDAR_ranges()
            #print "Aligning with obstacle"
            self.go(RIGHT)
            rospy.sleep(0.1)
        while not self.should_leave_wall():
            rospy.sleep(0.1)
            (front, left) = current_dists.get()
            if front <= WALL_PADDING:
                #self.print_LiDAR_ranges()
                #print "Still aligning with obstacle"
                self.go(RIGHT)
            elif WALL_PADDING - 0.5 <= left <= WALL_PADDING + .5:
                #print "Following obstacle edge"
                #self.print_LiDAR_ranges()
                self.go(STRAIGHT)
            elif left > WALL_PADDING + 0.5:
                #print "Getting too far away from obstacle"
                self.go(LEFT)
            else:
                #print "Aligning with obstacle again."
                self.go(RIGHT)
            
        # self.print_error()
        # print "Left Obstacle"

    def should_leave_wall(self):
        print "You dolt! You need to subclass bug to know how to leave the wall"
        sys.exit(0.1)

class Bug0(Bug):

    # If we are pointing towards the target location and the way is clear leave the obstacle
    def should_leave_wall(self):
        (x, y, t) = current_location.current_location()
        dir_to_go = current_location.global_to_local(necessary_heading(x, y, self.tx, self.ty))

        if abs(dir_to_go - t) < math.pi/4 and  current_dists.get()[0] > 5:
                #self.print_error()
                # print "Leaving obstacle"
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

def bug_algorithm(tx, ty, bug_type):

    
    # Track success stats
    success_count = 0
    success_distance = 0
    success_time = 0
    stats_printed = False
    
    print "Waiting for location data on '/scout_1/odom/filtered...'"
    rospy.wait_for_message('/scout_1/odom/filtered', Odometry,)
    print "... received."

    print("Waiting for break service...")
    rospy.wait_for_service('/scout_1/brake_rover')
    global brake_service
    brake_service = rospy.ServiceProxy('/scout_1/brake_rover', srv.BrakeRoverSrv)
    print("... active.")

    if bug_type == 0:
        bug = Bug0(tx,ty)
    elif bug_type == 1:
        bug = Bug1(tx,ty)
    elif bug_type == 2:
        bug = Bug2(tx,ty)
    else:
        print "Unknown Bug algorithm", bug_type
        sys.exit(3)
        
    # For status messages so other nodes know when we are done or if we failed
    status_topic = '/scout_1/bug_nav_status'
    bug_nav_status_publisher = rospy.Publisher(status_topic, String, queue_size=10)
    print "Publishing status messages on", status_topic

    # Add the command line waypoint to the queue
    waypoint_queue.put(Point(tx, ty, 0))
    
    random_waypoint_generator( max_num_waypoints )

    # Track total time spent
    total_time_start = rospy.get_rostime().secs
    
    
    # Check for new waypoints every 10 seconds
    idle = rospy.Rate(10)
    
    ###### main waypoint consumer loop  - run till node shuts down  ######
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        
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
            distance_to_cover = current_location.distance(wtx, wty)
            print "Moving to coordinates from waypoint:", (round(wtx,2), round(wty,2)), "Distance: ", round(distance_to_cover,2), "m."
            while current_location.distance(wtx, wty) > delta:
                try:
                    # These two functions are the heart of the algorithm. "Go_until_obstacle" moves towards the target location when there are no
                    # detected obstacles.
                    # The second (wall_follow) navigates around obstacles and positions the rover so that it can move towards the
                    # target location again
                    hit_wall = bug.go_until_obstacle()
                    if hit_wall:
                        bug.follow_wall()
                except TimedOutException:
                    elapsed_time = rospy.get_rostime().secs - start_time
                    print "Failed to reach",  (round(wtx,2), round(wty,2)), " after", round(elapsed_time), "(sim) seconds. Distance: ", round(current_location.distance(wtx, wty),2)
                    status_msg = "Timeout:", (wtx, wty)
                    bug_nav_status_publisher.publish(status_msg)

                    global timed_out
                    timed_out = False
                    break
                    
            # Confirm the target location was reached
            if current_location.distance(wtx, wty) < delta:
                elapsed_time = rospy.get_rostime().secs - start_time
                print "Arrived at", (round(wtx,2), round(wty,2)), " after", round(elapsed_time), "seconds. Distance: ", round(current_location.distance(wtx, wty),2)
                status_msg = "Arrived:", (wtx, wty)
                bug_nav_status_publisher.publish(status_msg)
                success_count += 1
                success_distance += distance_to_cover
                success_time += elapsed_time 
                
            bug.apply_brakes()
            print "There are", waypoint_queue.qsize(), "waypoints remaining."

        if not stats_printed:
            print "Succeeded: ", round((success_count/max_num_waypoints)*100), "% of the time."
            print "Distance covered: ", round(success_distance,2), "m"
            print "Time spent on successful runs: ", round(success_time,2), "s"
            print "Avg Speed: ", round(success_distance/success_time,2), "m/s"
            # Track total time spent
            total_time_elapsed = rospy.get_rostime().secs - total_time_start
            print "Total Time: ", round(total_time_elapsed,2), "s" 
    
            
            stats_printed = True
            
        idle.sleep()
       

def sigint_handler(signal_received, frame):
    print('SIGINT or CTRL-C received. Exiting.')
    exit(0)
        
# Parse arguments
algorithm = sys.argv[1]
algorithms = ["bug0", "bug1", "bug2"]
if algorithm not in algorithms:
    print "First argument should be one of ", algorithms, ". Was ", algorithm
    sys.exit(1)

if len(sys.argv) < 4:
    print "Usage: rosrun scoot bug_obstacle_nav.py bug<0|1|2> target_x target_y"
    sys.exit(1)
(command_tx, command_ty) = map(float, sys.argv[2:4])

signal(SIGINT, sigint_handler)

print('Running. Press CTRL-C to exit.')

init_listener()
 
print "Target given as argument:", (command_tx, command_ty)
bug = None
if algorithm == "bug0":
    bug_type = 0
elif algorithm == "bug1":
    bug_type = 1
elif algorithm == "bug2":
    bug_type = 2
bug_algorithm(command_tx, command_ty, bug_type)

