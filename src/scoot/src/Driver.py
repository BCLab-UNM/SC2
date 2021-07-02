#! /usr/bin/env python3 
import rospy
import angles
import math
import threading
from multiprocessing import Queue  # or import queue
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Point
from scoot.srv import Core
from scoot.msg import MoveResult
from obstacle.msg import Obstacles
from object_detection.msg import Detection
from Scoot import Location

driving_lock = threading.Lock()
control_lock = threading.Lock()


class Sync(object):
    def __init__(self, lock):
        self.lock = lock

    def __call__(self, func):
        def wrapped_f(*args, **kwargs):
            with self.lock:
                return func(*args, **kwargs)

        return wrapped_f


class Task:
    """A robot relative place to navigate to. Expressed as r and theta"""

    def __init__(self, msg, blocking=True):
        self.request = msg
        self.result = MoveResult.SUCCESS


def print_debug(msg):
    rospy.loginfo(msg)
    #     if self.dbg_msg is None or self.dbg_msg != msg:
    #         s = String()
    #         s.data = msg
    #         self.state_machine.publish(s)
    #     self.dbg_msg = msg


class State:
    """Global robot state variables"""

    STATE_IDLE = 0
    STATE_TURN = 1
    STATE_DRIVE = 2
    STATE_REVERSE = 3
    STATE_TIMED = 4

    DRIVE_MODE_STOP = 0
    DRIVE_MODE_PID = 1

    # Tune these with dynaimc reconfigure.
    DRIVE_SPEED = 0
    DRIVE_SPEED_SLOW = 0
    REVERSE_SPEED = 0
    TURN_SPEED = 0
    TURN_SPEED_SLOW = 0
    HEADING_RESTORE_FACTOR = 0
    GOAL_DISTANCE_OK = 0
    ROTATE_THRESHOLD = 0
    DRIVE_ANGLE_ABORT = 0

    DRIVE_SPEED_MAX = 2 * math.pi
    TURN_SPEED_MAX = 1.2

    def __init__(self):
        self.MapLocation = Location(None)
        self.OdomLocation = Location(None)
        self.CurrentState = State.STATE_IDLE
        self.Goal = None
        self.Start = None
        self.TimerCount = 0
        self.Doing = None
        self.Work = Queue()
        self.dbg_msg = None
        self.current_obstacles = 0
        self.current_obstacle_data = 0
        self.current_distance = float('inf')
        self.obstacle_heading = 0
        self.task = MoveResult.SUCCESS
        self.last_twist = Twist(linear=[math.inf, math.inf])

        self.rover_name = rospy.get_param('rover_name', default='small_scout_1')

        # Configuration 
        State.DRIVE_SPEED = rospy.get_param("/" + self.rover_name + "/Core/DRIVE_SPEED", default=5)
        State.DRIVE_SPEED_SLOW = rospy.get_param("/" + self.rover_name + "/Core/DRIVE_SPEED_SLOW", default=1)
        State.REVERSE_SPEED = rospy.get_param("/" + self.rover_name + "/Core/REVERSE_SPEED", default=5)
        State.TURN_SPEED = rospy.get_param("/" + self.rover_name + "/Core/TURN_SPEED", default=5)
        State.TURN_SPEED_SLOW = rospy.get_param("/" + self.rover_name + "/Core/TURN_SPEED_SLOW", default=5)
        State.HEADING_RESTORE_FACTOR = rospy.get_param("/" + self.rover_name + "/Core/HEADING_RESTORE_FACTOR",
                                                       default=2)
        State.GOAL_DISTANCE_OK = rospy.get_param("/" + self.rover_name + "/Core/GOAL_DISTANCE_OK", default=0.1)
        State.ROTATE_THRESHOLD = rospy.get_param("/" + self.rover_name + "/Core/ROTATE_THRESHOLD", default=math.pi / 32)
        State.DRIVE_ANGLE_ABORT = rospy.get_param("/" + self.rover_name + "/Core/DRIVE_ANGLE_ABORT",
                                                  default=math.pi / 4)

        # Subscribers
        rospy.Subscriber('/' + self.rover_name + '/obstacle', Obstacles, self._obstacle)
        rospy.Subscriber('/' + self.rover_name + '/detections', Detection, self._vision)
        rospy.Subscriber('/' + self.rover_name + '/odometry/filtered', Odometry, self._odom)

        # Services 
        self.control = rospy.Service('control', Core, self._control)

        # Publishers
        self.driveControl = rospy.Publisher('/' + self.rover_name + '/cmd_vel', Twist, queue_size=10)

        # Configuration 
        # self.config_srv = Server(driveConfig, self._reconfigure)

    def _stop_now(self, result):
        print_debug('IDLE')
        self.drive(0, 0, 0, State.DRIVE_MODE_STOP)
        self.CurrentState = State.STATE_IDLE
        while not self.Work.empty():
            item = self.Work.get(False)
            item.result = result
        self.task.result = result
        if self.Doing is not None:
            self.Doing.result = result

    @Sync(control_lock)
    def _control(self, req):
        rospy.loginfo("_control: Called:" + str(req))
        self.current_distance = float('inf')
        self.current_obstacles = 0
        self.current_obstacle_data = 0
        for r in req.req[:-1]:
            self.Work.put(Task(r, False), False)

        r = req.req[-1]
        self.task = Task(r, True)
        self.Work.put(self.task, True)  # @TODO think about if we do want to support a drive queue

        sleep_wait = 0.2
        sleep_turns = r.timeout / sleep_wait

        rospy.sleep(sleep_wait)
        self.run()  # this will be a blocking call
        rval = MoveResult()
        rval.result = self.task.result
        rval.obstacle = self.current_obstacles
        rval.obstacle_data = self.current_obstacle_data
        rval.distance = self.current_distance
        rval.heading = self.obstacle_heading
        self.current_obstacles = 0
        self.current_obstacle_data = 0
        self.current_distance = float('inf')
        self.obstacle_heading = 0
        rospy.loginfo("_control: right before return")
        rospy.loginfo("_control rval:" + str(rval))
        return rval

    def _reconfigure(self, config, level):
        State.DRIVE_SPEED = config["DRIVE_SPEED"]
        State.REVERSE_SPEED = config["REVERSE_SPEED"]
        State.TURN_SPEED = config["TURN_SPEED"]
        State.HEADING_RESTORE_FACTOR = config["HEADING_RESTORE_FACTOR"]
        State.GOAL_DISTANCE_OK = config["GOAL_DISTANCE_OK"]
        State.ROTATE_THRESHOLD = config["ROTATE_THRESHOLD"]
        State.DRIVE_ANGLE_ABORT = config["DRIVE_ANGLE_ABORT"]
        print_debug('Mobility parameter reconfiguration done.')
        return config

    def set_mode(self, msg):
        if msg.data == 1:
            self._stop_now(MoveResult.USER_ABORT)

    def __check_obstacles(self):
        if self.Doing is not None:
            detected = self.current_obstacles & self.Doing.request.obstacles

            if (detected & Obstacles.IS_LIDAR) != 0:
                self._stop_now(MoveResult.OBSTACLE_LASER)
                print_debug("__check_obstacles: MoveResult.OBSTACLE_LASER")
            elif (detected & Obstacles.IS_VOLATILE) != 0:
                self._stop_now(MoveResult.OBSTACLE_VOLATILE)
                print_debug("__check_obstacles: MoveResult.OBSTACLE_VOLATILE")
            elif (detected & Obstacles.VISION_VOLATILE) != 0:
                self._stop_now(MoveResult.VISION_VOLATILE)
                print_debug("__check_obstacles: MoveResult.VISION_VOLATILE")
            elif (detected & Obstacles.CUBESAT) != 0:
                self._stop_now(MoveResult.CUBESAT)
                print_debug("__check_obstacles: MoveResult.CUBESAT")
            elif (detected & Obstacles.HOME_LEG) != 0:
                self._stop_now(MoveResult.HOME_LEG)
                print_debug("__check_obstacles: MoveResult.HOME_LEG")
            elif (detected & Obstacles.HOME_FIDUCIAL) != 0:
                self._stop_now(MoveResult.HOME_FIDUCIAL)
                print_debug("__check_obstacles: MoveResult.HOME_FIDUCIAL")

    def _obstacle(self, msg):
        self.current_obstacles &= ~msg.mask
        self.current_obstacles |= msg.msg
        if self.current_obstacles & Obstacles.IS_LIDAR:
            if self.current_distance > msg.distance:
                self.current_distance = msg.distance
        else:
            self.current_obstacle_data = msg.data
        self.__check_obstacles()

    def _vision(self, msg):
        rospy.loginfo("Driver.py's _vision called with:")
        rospy.loginfo(msg)
        self.current_obstacles |= msg.detection_id
        self.current_distance = msg.distance
        self.obstacle_heading = msg.heading
        if msg.detection_id == Obstacles.CUBESAT:
            rospy.set_param("/" + self.rover_name + "/cubesat_point_from_rover", {'x': msg.x, 'y': msg.y, 'z': msg.z})
        self.__check_obstacles()

    def _odom(self, msg):
        self.OdomLocation.Odometry = msg

    @Sync(driving_lock)
    def drive(self, linear_x, linear_y, angular, mode):
        if mode is State.DRIVE_MODE_STOP:
            self.Doing = None
        t = Twist()
        t.linear.x = linear_x
        t.linear.y = linear_y
        t.angular.y = mode
        t.angular.z = angular
        if math.isclose(t.angular.z, 0, abs_tol=.1):  # practically 0
            t.angular.z = 0
        if self.last_twist != t:
            print_debug('drive called: x: ' + str(linear_x))
            print_debug('drive called: y: ' + str(linear_y))
            self.driveControl.publish(t)
        self.last_twist = t

    def run(self):
        r = rospy.Rate(10)  # 10hz
        while self.Doing or not self.Work.empty():
            r.sleep()
            if self.CurrentState == State.STATE_IDLE:
                print_debug('IDLE')
                if self.Work.empty():
                    self.Doing = None
                if not self.Work.empty():
                    self.Doing = self.Work.get(False)
                    if self.Doing:
                        if self.Doing.request.timer > 0:
                            self.TimerCount = self.Doing.request.timer * 10
                            self.CurrentState = State.STATE_TIMED
                        else:
                            self.CurrentState = State.STATE_DRIVE
                            req_theta = self.Doing.request.theta
                            cur = self.OdomLocation.get_pose()
                            self.Start = cur
                            self.Goal = Pose2D()
                            goal_dist = math.inf
                            if req_theta == 0:
                                self.Doing.request.linear = State.DRIVE_SPEED
                                if self.Doing.request.y == 0:  # Forward Backwards drive
                                    self.Goal.theta = cur.theta + req_theta
                                    self.Goal.x = cur.x + self.Doing.request.x * math.cos(self.Goal.theta)
                                    self.Goal.y = cur.y + self.Doing.request.x * math.sin(self.Goal.theta)
                                else:  # Strafing
                                    self.Goal.theta = cur.theta + req_theta  # @TODO check the math here
                                    self.Goal.x = cur.x + self.Doing.request.x * math.cos(
                                        cur.theta) + self.Doing.request.y * math.sin(cur.theta)
                                    self.Goal.y = cur.y + self.Doing.request.x * math.sin(
                                        cur.theta) + self.Doing.request.y * math.cos(cur.theta)
                            elif abs(req_theta) > State.ROTATE_THRESHOLD:  # Turn
                                self.CurrentState = State.STATE_TURN
                                self.Goal.theta = cur.theta + req_theta
                                if self.Doing.request.angular > State.TURN_SPEED_MAX:
                                    self.Doing.request.angular = State.TURN_SPEED_MAX
                                elif self.Doing.request.angular <= 0:
                                    self.Doing.request.angular = State.TURN_SPEED
                        self.__check_obstacles()
            if self.Doing:
                if self.CurrentState == State.STATE_TURN:
                    print_debug('TURN')
                    self.__check_obstacles()
                    cur = self.OdomLocation.get_pose()
                    heading_error = angles.shortest_angular_distance(cur.theta, self.Goal.theta)
                    print_debug('heading_error:' + str(heading_error))
                    print_debug("State.ROTATE_THRESHOLD:" + str(State.ROTATE_THRESHOLD))
                    if math.isclose(heading_error, 0, abs_tol=0.2):
                        self.Doing.request.angular = State.TURN_SPEED_SLOW  # CLOSE so slowing down
                    if math.isclose(heading_error, 0, abs_tol=0.1):
                        self.Doing.request.angular = State.TURN_SPEED_SLOW / 4  # SOOO CLOSE so slowing down more
                    if abs(heading_error) > State.ROTATE_THRESHOLD and not math.isclose(heading_error, 0,
                                                                                        abs_tol=0.001):
                        if heading_error < 0:
                            self.drive(0, 0, -self.Doing.request.angular, State.DRIVE_MODE_PID)
                        else:
                            self.drive(0, 0, self.Doing.request.angular, State.DRIVE_MODE_PID)
                    else:
                        self.Goal = None
                        self.CurrentState = State.STATE_IDLE
                        self.drive(0, 0, 0, State.DRIVE_MODE_STOP)
                elif self.CurrentState == State.STATE_DRIVE:
                    if self.Doing.request.y != 0:
                        print_debug('STRAFE')
                    else:
                        print_debug('DRIVE')
                    if self.Doing.request.y < 0:
                        print_debug('REVERSE Y')
                        # direction_y = -1

                    self.__check_obstacles()
                    cur = self.OdomLocation.get_pose()
                    heading_error = angles.shortest_angular_distance(cur.theta, self.Goal.theta)
                    goal_angle = angles.shortest_angular_distance(math.pi + cur.theta,
                                                                  math.atan2(self.Goal.y - cur.y, self.Goal.x - cur.x))
                    new_goal_dist = math.hypot(self.Goal.x - self.OdomLocation.Odometry.pose.pose.position.x,
                                               self.Goal.y - self.OdomLocation.Odometry.pose.pose.position.y)
                    if goal_dist - new_goal_dist < -State.GOAL_DISTANCE_OK:  # overshot allowed distance
                        print_debug('overshot: ' + str(new_goal_dist))
                        self.drive(0, 0, 0,
                                   State.DRIVE_MODE_PID)  # for waiting use DRIVE_MODE_PID so we dont clear the task
                        r.sleep()
                        new_goal_dist = math.hypot(self.Goal.x - self.OdomLocation.Odometry.pose.pose.position.x,
                                                   self.Goal.y - self.OdomLocation.Odometry.pose.pose.position.y)
                        if new_goal_dist <= State.GOAL_DISTANCE_OK:  # stopped and double check
                            print_debug('at goal')
                            self.Goal = None
                            self.CurrentState = State.STATE_IDLE
                            self.drive(0, 0, 0, State.DRIVE_MODE_STOP)
                        else:
                            print_debug('overshot: stopping @TODO FIX ME')
                            self.Goal = None
                            self.CurrentState = State.STATE_IDLE
                            self.drive(0, 0, 0, State.DRIVE_MODE_STOP)
                            """ @TODO Fix & finish this code so if overshoot can set a new goal or if exceeds PATH_FAIL
                            if not math.isclose(self.Goal.x - self.OdomLocation.Odometry.pose.pose.position.x, 0,
                                                abs_tol=State.GOAL_DISTANCE_OK):
                                # if self.Goal.x - self.OdomLocation.Odometry.pose.pose.position.x > 0:
                                self.Doing.request.x = self.Goal.x - self.OdomLocation.Odometry.pose.pose.position.x
                                self.Doing.request.linear = State.DRIVE_SPEED_SLOW  # State.DRIVE_SPEED / 4
                                print_debug('overshot recalc x' + str(self.Doing.request.x))
                            if not math.isclose(self.Goal.y - self.OdomLocation.Odometry.pose.pose.position.y, 0,
                                                abs_tol=State.GOAL_DISTANCE_OK):
                                self.Doing.request.y = self.Goal.y - self.OdomLocation.Odometry.pose.pose.position.y
                                self.Doing.request.linear = State.DRIVE_SPEED_SLOW  # State.DRIVE_SPEED / 4
                                print_debug('overshot recalc y' + str(self.Doing.request.y))
                            """
                    elif self.OdomLocation.at_goal(self.Goal, State.GOAL_DISTANCE_OK):
                        print_debug('at goal')
                        self.Goal = None
                        self.CurrentState = State.STATE_IDLE
                        self.drive(0, 0, 0, State.DRIVE_MODE_STOP)
                    elif self.Doing.request.y == 0 and abs(heading_error) > State.DRIVE_ANGLE_ABORT / 2:
                        print_debug('heading_error exceeded')
                        self.Goal = None
                        self.Doing.result = MoveResult.PATH_FAIL
                        self.CurrentState = State.STATE_IDLE
                        self._stop_now(MoveResult.PATH_FAIL)
                        self.drive(0, 0, 0, State.DRIVE_MODE_STOP)
                    else:
                        if abs(self.Doing.request.y) + abs(self.Doing.request.x) == 0:
                            linear_x = 0
                        else:
                            linear_x = self.Doing.request.linear * self.Doing.request.x / (
                                    abs(self.Doing.request.y) + abs(self.Doing.request.x))
                        if abs(self.Doing.request.y) + abs(self.Doing.request.x) == 0:
                            linear_y = 0
                        else:
                            linear_y = self.Doing.request.linear * self.Doing.request.y / (
                                    abs(self.Doing.request.y) + abs(self.Doing.request.x))
                        self.drive(linear_x, linear_y, heading_error * State.HEADING_RESTORE_FACTOR,
                                   State.DRIVE_MODE_PID)
                    goal_dist = new_goal_dist

                elif self.CurrentState == State.STATE_TIMED:
                    print_debug('TIMED')
                    self.__check_obstacles()
                    if self.Doing.request.linear == 0 and self.Doing.request.angular == 0:
                        self.drive(0, 0, 0, State.DRIVE_MODE_STOP)
                    else:
                        self.drive(self.Doing.request.linear, 0, self.Doing.request.angular,
                                   State.DRIVE_MODE_PID)

                    if self.TimerCount == 0:
                        self.CurrentState = State.STATE_IDLE
                        self.drive(0, 0, 0, State.DRIVE_MODE_STOP)
                    else:
                        self.TimerCount = self.TimerCount - 1
