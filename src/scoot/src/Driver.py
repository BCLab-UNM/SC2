#! /usr/bin/env python3 
import rospy
import angles
import math
import _thread
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
        self.sema = None
        """
        if blocking:
            self.sema = threading.Semaphore(0)
        else:
            self.sema = None
        """


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
    REVERSE_SPEED = 0
    TURN_SPEED = 0
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
        # self.JoystickCommand = Joy()
        # self.JoystickCommand.axes = [0,0,0,0,0,0]

        self.rover_name = rospy.get_param('rover_name', default='small_scout_1')

        # Configuration 
        State.DRIVE_SPEED = rospy.get_param("/" + self.rover_name + "/Core/DRIVE_SPEED", default=5)
        State.REVERSE_SPEED = rospy.get_param("/" + self.rover_name + "/Core/REVERSE_SPEED", default=5)
        State.TURN_SPEED = rospy.get_param("/" + self.rover_name + "/Core/TURN_SPEED", default=5)
        State.HEADING_RESTORE_FACTOR = rospy.get_param("/" + self.rover_name + "/Core/HEADING_RESTORE_FACTOR",
                                                       default=2)
        State.GOAL_DISTANCE_OK = rospy.get_param("/" + self.rover_name + "/Core/GOAL_DISTANCE_OK", default=0.1)
        State.ROTATE_THRESHOLD = rospy.get_param("/" + self.rover_name + "/Core/ROTATE_THRESHOLD", default=math.pi / 16)
        State.DRIVE_ANGLE_ABORT = rospy.get_param("/" + self.rover_name + "/Core/DRIVE_ANGLE_ABORT",
                                                  default=math.pi / 4)

        # Subscribers
        # rospy.Subscriber('joystick', Joy, self._joystick, queue_size=10)
        rospy.Subscriber('/' + self.rover_name + '/obstacle', Obstacles, self._obstacle)
        rospy.Subscriber('/' + self.rover_name + '/detections', Detection, self._vision)
        rospy.Subscriber('/' + self.rover_name + '/odometry/filtered', Odometry, self._odom)

        # Services 
        self.control = rospy.Service('control', Core, self._control)

        # Publishers
        # self.state_machine = rospy.Publisher('state_machine', String, queue_size=1, latch=True)
        self.driveControl = rospy.Publisher('/' + self.rover_name + '/cmd_vel', Twist, queue_size=10)

        # Configuration 
        # self.config_srv = Server(driveConfig, self._reconfigure)

        # Start a thread to do initial configuration.
        _thread.start_new_thread(self.do_initial_config, ())

    def _stop_now(self, result):
        print_debug('IDLE')
        self.drive(0, 0, State.DRIVE_MODE_STOP)
        self.CurrentState = State.STATE_IDLE
        while not self.Work.empty():
            item = self.Work.get(False)
            item.result = result
            if item.sema is not None:
                item.sema.release()

        if self.Doing is not None:
            self.Doing.result = result

    @Sync(control_lock)
    def _control(self, req):
        rospy.loginfo("_control: Called")
        self.current_distance = float('inf')
        self.current_obstacles = 0
        self.current_obstacle_data = 0
        for r in req.req[:-1]:
            self.Work.put(Task(r, False), False)

        r = req.req[-1]
        t = Task(r, True)
        self.Work.put(t, True)

        sleep_wait = 0.2
        sleep_turns = r.timeout / sleep_wait
        while not self.Work.empty() or self.Doing is not None or self.Goal is not None:
            with driving_lock:  # try to stay waiting until a drive is finished
                pass
            rospy.sleep(sleep_wait)
            # sleep_turns -= 1
            # if sleep_turns == 0:
            #        with driving_lock:
            #                self._stop_now(MoveResult.TIMEOUT)

        rval = MoveResult()
        rval.result = t.result
        rval.obstacle = self.current_obstacles
        rval.obstacle_data = self.current_obstacle_data
        rval.distance = self.current_distance
        rval.heading = self.obstacle_heading
        self.current_obstacles = 0
        self.current_obstacle_data = 0
        self.current_distance = float('inf')
        self.obstacle_heading = 0
        rospy.loginfo("_control: right before return")
        return rval

    @Sync(driving_lock)
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

        # @sync(driving_lock)

    # def _joystick(self, joy_command):
    # self.JoystickCommand = joy_command

    @Sync(driving_lock)
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

    @Sync(driving_lock)
    def _obstacle(self, msg):
        self.current_obstacles &= ~msg.mask
        self.current_obstacles |= msg.msg
        if self.current_obstacles & Obstacles.IS_LIDAR:
            if self.current_distance > msg.distance:
                self.current_distance = msg.distance
        else:
            self.current_obstacle_data = msg.data
        self.__check_obstacles()

    @Sync(driving_lock)
    def _vision(self, msg):
        rospy.loginfo("Driver.py's _vision called with:")
        rospy.loginfo(msg)
        self.current_obstacles |= msg.detection_id
        self.current_distance = msg.distance
        self.obstacle_heading = msg.heading
        if msg.detection_id == Obstacles.CUBESAT:
            rospy.set_param("/" + self.rover_name + "/cubesat_point_from_rover", {'x': msg.x, 'y': msg.y, 'z': msg.z})
        self.__check_obstacles()

    @Sync(driving_lock)
    def _odom(self, msg):
        self.OdomLocation.Odometry = msg

    def drive(self, linear, angular, mode):
        t = Twist()
        t.linear.x = linear
        t.angular.y = mode
        t.angular.z = angular
        self.driveControl.publish(t)

    @Sync(driving_lock)
    def run(self):
        if self.CurrentState == State.STATE_IDLE:
            # self.print_debug('IDLE')
            if self.Doing is not None:
                if self.Doing.sema is not None:
                    self.Doing.sema.release()
                self.Doing = None

            if self.Work.empty():
                pass
                '''
                # Let the joystick drive.
                lin = self.JoystickCommand.axes[4] * State.DRIVE_SPEED
                ang = self.JoystickCommand.axes[3] * State.TURN_SPEED
                if abs(lin) < 0.1 and abs(ang) < 0.1 :
                    self.drive(0, 0, State.DRIVE_MODE_STOP)
                else:
                    self.drive(lin, ang, State.DRIVE_MODE_PID)
                 '''
            else:
                self.Doing = self.Work.get(False)

                if self.Doing.request.timer > 0:
                    self.TimerCount = self.Doing.request.timer * 10
                    self.CurrentState = State.STATE_TIMED
                else:
                    if self.Doing.request.r < 0:
                        self.Doing.request.theta = 0

                    req_theta = self.Doing.request.theta
                    if req_theta > 0:
                        req_theta += State.ROTATE_THRESHOLD / 2.0
                    elif req_theta < 0:
                        req_theta -= State.ROTATE_THRESHOLD / 2.0

                    req_r = self.Doing.request.r
                    if req_r > 0:
                        req_r += State.GOAL_DISTANCE_OK / 2.0
                    elif req_r < 0:
                        req_r -= State.GOAL_DISTANCE_OK / 2.0

                    if self.Doing.request.linear > State.DRIVE_SPEED_MAX:
                        self.Doing.request.linear = State.DRIVE_SPEED_MAX
                    elif self.Doing.request.linear <= 0:
                        self.Doing.request.linear = State.DRIVE_SPEED

                    if self.Doing.request.angular > State.TURN_SPEED_MAX:
                        self.Doing.request.angular = State.TURN_SPEED_MAX
                    elif self.Doing.request.angular <= 0:
                        self.Doing.request.angular = State.TURN_SPEED

                    cur = self.OdomLocation.get_pose()
                    self.Goal = Pose2D()
                    self.Goal.theta = cur.theta + req_theta
                    self.Goal.x = cur.x + req_r * math.cos(self.Goal.theta)
                    self.Goal.y = cur.y + req_r * math.sin(self.Goal.theta)
                    self.Start = cur

                    if self.Doing.request.r < 0:
                        self.CurrentState = State.STATE_REVERSE
                    else:
                        self.CurrentState = State.STATE_TURN

                self.__check_obstacles()
        elif self.CurrentState == State.STATE_TURN:
            print_debug('TURN')
            self.__check_obstacles()
            cur = self.OdomLocation.get_pose()
            heading_error = angles.shortest_angular_distance(cur.theta, self.Goal.theta)
            if abs(heading_error) > State.ROTATE_THRESHOLD:
                if heading_error < 0:
                    self.drive(0, -self.Doing.request.angular, State.DRIVE_MODE_PID)
                else:
                    self.drive(0, self.Doing.request.angular, State.DRIVE_MODE_PID)
            else:
                self.CurrentState = State.STATE_DRIVE
                self.drive(0, 0, State.DRIVE_MODE_STOP)

        elif self.CurrentState == State.STATE_DRIVE:
            print_debug('DRIVE')
            self.__check_obstacles()
            cur = self.OdomLocation.get_pose()
            heading_error = angles.shortest_angular_distance(cur.theta, self.Goal.theta)
            goal_angle = angles.shortest_angular_distance(cur.theta,
                                                          math.atan2(self.Goal.y - cur.y, self.Goal.x - cur.x))
            if self.OdomLocation.at_goal(self.Goal, State.GOAL_DISTANCE_OK) or abs(
                    goal_angle) > State.DRIVE_ANGLE_ABORT:
                self.Goal = None
                self.CurrentState = State.STATE_IDLE
                self.drive(0, 0, State.DRIVE_MODE_STOP)

            elif abs(heading_error) > State.DRIVE_ANGLE_ABORT / 2:
                self._stop_now(MoveResult.PATH_FAIL)
                self.drive(0, 0, State.DRIVE_MODE_STOP)
            else:
                self.drive(self.Doing.request.linear,
                           heading_error * State.HEADING_RESTORE_FACTOR,
                           State.DRIVE_MODE_PID)

        elif self.CurrentState == State.STATE_REVERSE:
            print_debug('REVERSE')
            self.__check_obstacles()
            cur = self.OdomLocation.get_pose()
            heading_error = angles.shortest_angular_distance(cur.theta, self.Goal.theta)
            goal_angle = angles.shortest_angular_distance(math.pi + cur.theta,
                                                          math.atan2(self.Goal.y - cur.y, self.Goal.x - cur.x))
            if self.OdomLocation.at_goal(self.Goal, State.GOAL_DISTANCE_OK) or abs(
                    goal_angle) > State.DRIVE_ANGLE_ABORT:
                self.Goal = None
                self.CurrentState = State.STATE_IDLE
                self.drive(0, 0, State.DRIVE_MODE_STOP)
            elif abs(heading_error) > State.DRIVE_ANGLE_ABORT / 2:
                self._stop_now(MoveResult.PATH_FAIL)
                self.drive(0, 0, State.DRIVE_MODE_STOP)
            else:
                self.drive(-State.REVERSE_SPEED,
                           heading_error * State.HEADING_RESTORE_FACTOR,
                           State.DRIVE_MODE_PID)

        elif self.CurrentState == State.STATE_TIMED:
            print_debug('TIMED')
            self.__check_obstacles()
            if self.Doing.request.linear == 0 and self.Doing.request.angular == 0:
                self.drive(0, 0, State.DRIVE_MODE_STOP)
            else:
                self.drive(self.Doing.request.linear, self.Doing.request.angular, State.DRIVE_MODE_PID)

            if self.TimerCount == 0:
                self.CurrentState = State.STATE_IDLE
                self.drive(0, 0, State.DRIVE_MODE_STOP)
            else:
                self.TimerCount = self.TimerCount - 1

    def do_initial_config(self):
        # Do initial configuration. 
        params = {
            "DRIVE_SPEED": State.DRIVE_SPEED,
            "REVERSE_SPEED": State.REVERSE_SPEED,
            "TURN_SPEED": State.TURN_SPEED,
            "HEADING_RESTORE_FACTOR": State.HEADING_RESTORE_FACTOR,
            "GOAL_DISTANCE_OK": State.GOAL_DISTANCE_OK,
            "ROTATE_THRESHOLD": State.ROTATE_THRESHOLD,
            "DRIVE_ANGLE_ABORT": State.DRIVE_ANGLE_ABORT,
        }
        # dyn_client = Client('scoot')
        # dyn_client.update_configuration(params)
        print('Initial configuration sent.')
