#!/usr/bin/env python3
import rospy
import math
import angles
import tf
import threading

from rospy import ServiceException
from srcp2_msgs import msg, srv

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D, Point, PoseWithCovarianceStamped, PoseStamped, \
    Quaternion, Pose
from nav_msgs.msg import Odometry
from scoot.msg import MoveResult, MoveRequest
from scoot.srv import Core

odom_lock = threading.Lock()
drive_lock = threading.Lock()
joint_lock = threading.Lock()
health_lock = threading.Lock()


class DriveException(Exception):
    def __init__(self, st):
        self.status = st


class VisionException(DriveException):
    pass


class VolatileException(DriveException):
    pass


class ObstacleException(DriveException):
    def __init__(self, st, obstacle, distance):
        self.status = st
        self.obstacle = obstacle
        self.distance = distance


class CubesatException(VisionException):
    def __init__(self, heading, distance, point):
        self.heading = heading
        self.distance = distance
        self.point = point


class VisionVolatileException(VisionException):
    def __init__(self, heading, distance):
        self.heading = heading
        self.distance = distance


class HomeLegException(VisionException):
    def __init__(self, heading, distance):
        self.heading = heading
        self.distance = distance


class HomeLogoException(VisionException):
    def __init__(self, heading, distance):
        self.heading = heading
        self.distance = distance


class PathException(DriveException):
    pass


class AbortException(DriveException):
    pass


class TimeoutException(DriveException):
    pass


class Sync(object):
    def __init__(self, lock):
        self.lock = lock

    def __call__(self, func):
        def wrapped_f(*args, **kwargs):
            with self.lock:
                return func(*args, **kwargs)

        return wrapped_f


class Location:
    """A class that encodes an EKF provided location and accessor methods"""

    def __init__(self, odo):
        self.Odometry = odo

    def get_pose(self):  # TODO: add a ros warning if self.Odometry none
        """Return a std_msgs.Pose from this Location. Useful because Pose
        has angles represented as roll, pitch, yaw.

        Returns:

        * (`std_msgs.msg.Pose`) The pose.
        """
        if self.Odometry is None:
            quat = [0, 0, 0, 0, ]
        else:
            quat = [self.Odometry.pose.pose.orientation.x,
                    self.Odometry.pose.pose.orientation.y,
                    self.Odometry.pose.pose.orientation.z,
                    self.Odometry.pose.pose.orientation.w,
                    ]
        (r, p, y) = tf.transformations.euler_from_quaternion(quat)

        pose = Pose2D()
        if self.Odometry is None:
            pose.x = 0
            pose.y = 0
        else:
            pose.x = self.Odometry.pose.pose.position.x
            pose.y = self.Odometry.pose.pose.position.y
        pose.theta = y

        return pose

    def at_goal(self, goal, distance):
        """Determine if the pose is within acceptable distance of this location

        Returns:

        * (`bool`) True if within the target distance.
        """
        dist = math.hypot(goal.x - self.Odometry.pose.pose.position.x,
                          goal.y - self.Odometry.pose.pose.position.y)

        return dist < distance


class Scoot(object):
    """Class that embodies the Scoot's API

    This is the Python API used to drive the rover. The API interfaces
    with ROS topics and services to perform action and acquire sensor data.
    """

    def __init__(self, rover):
        self.rover_name = rover  # @note this will get overwritten but should still be correct
        self.rover_type = self.rover_name.split("_")[1]  # @note this will get overwritten but should still be correct
        self.TURN_SPEED = 0
        self.DRIVE_SPEED = 0
        self.REVERSE_SPEED = 0
        self.VOL_TYPES = None
        self.health = None

        self.sensor_pitch_control_topic = None
        self.sensor_yaw_control_topic = None
        self.shoulder_yaw_control = None
        self.shoulder_pitch_control = None
        self.bucket_control = None
        self.elbow_pitch_control = None
        self.bin_control = None

        self.bucket_info_msg = None
        self.bin_info_msg = None

        self.localization_service = None
        self.model_state_service = None
        self.vol_list_service = None
        self.power_saver_service = None
        self.power_saver_state = False

        self.truePoseCalled = False
        self.true_pose_got = None

        self.OdomLocation = Location(None)
        self.repair_station_pose = Point()
        self.world_offset = None
        self.control = None
        self.control_data = None
        self.dist_data = None
        self.joint_states = None
        self.xform = None
        self.vol_delay = 0

        self.cubesat_point = Point(0, 0, 0)
        self.cubesat_found = False
        self.home_arrived = False
        self.home_logo_found = False
        self.score = None

    def start(self, **kwargs):
        if 'tf_rover_name' in kwargs:
            self.rover_name = kwargs['tf_rover_name']
        else:
            self.rover_name = rospy.get_namespace()
        self.rover_name = self.rover_name.strip('/')
        self.rover_type = self.rover_name.split("_")[1]  # cuts of the _# part of the rover name
        self.DRIVE_SPEED = rospy.get_param("/" + self.rover_name + "/Core/DRIVE_SPEED", default=5)
        self.REVERSE_SPEED = rospy.get_param("/" + self.rover_name + "/Core/REVERSE_SPEED", default=5)
        self.TURN_SPEED = rospy.get_param("/" + self.rover_name + "/Core/TURN_SPEED", default=5)

        '''Tracking SRCP2's Wiki 
                Documentation/API/Robots/Hauler.md  
                Documentation/Qualification-Rounds/Round-Two.md 
                Documentation/Qualification-Rounds/Round-Two.md d
            HaulerMsg.msg's comment in srcp2-competitors/ros_workspace/install/shares/rcp2_msgs/msg/
            our scoot.launch
            and matching with indexes from our Obstacles.msg '''
        self.VOL_TYPES = rospy.get_param("vol_types",
                                         default=["ice", "ethane", "methane", "methanol", "carbon_dioxide", "ammonia",
                                                  "hydrogen_sulfite", "sulfur_dioxide", "regolith"])

        #  @NOTE: when we use namespaces we wont need to have the rover_name
        # Create publishers.
        self.sensor_pitch_control_topic = rospy.Publisher('/' + self.rover_name + '/sensor/pitch/command/position',
                                                          Float64, queue_size=10)
        self.sensor_yaw_control_topic = rospy.Publisher('/' + self.rover_name + '/sensor/yaw/command/position', Float64,
                                                        queue_size=10)
        # Connect to services.
        rospy.loginfo("Waiting for control service")
        rospy.wait_for_service('/' + self.rover_name + '/control')
        self.control = rospy.ServiceProxy('/' + self.rover_name + '/control', Core)
        rospy.loginfo("Done waiting for control service")

        rospy.wait_for_service('/' + self.rover_name + '/get_true_pose')
        self.localization_service = rospy.ServiceProxy('/' + self.rover_name + '/get_true_pose', srv.LocalizationSrv)
        rospy.wait_for_service('/' + self.rover_name + '/system_monitor/power_saver')
        self.power_saver_service = rospy.ServiceProxy('/' + self.rover_name + '/system_monitor/power_saver',
                                                      srv.SystemPowerSaveSrv)

        rospy.loginfo("Done waiting for general services")
        if self.rover_type == "scout":
            pass  # rospy.wait_for_service('/vol_detected_service')
        elif self.rover_type == "excavator":
            self.shoulder_yaw_control = rospy.Publisher('/' + self.rover_name + '/arm/shoulder_yaw/command/position',
                                                        Float64,
                                                        queue_size=10)
            self.shoulder_pitch_control = rospy.Publisher(
                '/' + self.rover_name + '/arm/shoulder_pitch/command/position',
                Float64, queue_size=10)
            self.bucket_control = rospy.Publisher('/' + self.rover_name + '/arm/wrist_pitch/command/position', Float64,
                                                  queue_size=10)
            self.elbow_pitch_control = rospy.Publisher('/' + self.rover_name + '/arm/elbow_pitch/command/position',
                                                       Float64, queue_size=10)

            rospy.Subscriber('/' + self.rover_name + '/scoop_info', msg.ExcavatorScoopMsg, self._bucket_info)

        elif self.rover_type == "hauler":
            self.bin_control = rospy.Publisher('/' + self.rover_name + '/bin/command/position', Float64,
                                               queue_size=10)
            # rospy.Subscriber('/' + self.rover_name + '/bin_info', msg.HaulerMsg, self._bin_info)

        rospy.loginfo("Done waiting for rover specific services")
        # Subscribe to topics.
        rospy.Subscriber('/' + self.rover_name + '/odometry/filtered', Odometry, self._odom)
        rospy.Subscriber('/' + self.rover_name + '/system_monitor', msg.SystemMonitorMsg, self._health)
        rospy.Subscriber('/' + self.rover_name + '/joint_states', JointState, self._joint_states)
        rospy.Subscriber('/srcp2/score', msg.ScoreMsg, self._score)
        # Transform listener. Use this to transform between coordinate spaces.
        # Transform messages must predate any sensor messages so initialize this first.
        self.xform = tf.TransformListener()
        rospy.loginfo("Scoot Ready")

    @Sync(odom_lock)
    def _odom(self, message):
        self.OdomLocation.Odometry = message

    @Sync(health_lock)
    def _health(self, message):
        self.health = message

    def _bin_info(self, message):
        self.bin_info_msg = message

    def _bucket_info(self, message):
        self.bucket_info_msg = message

    def _joint_states(self, message):
        self.joint_states = message

    def _score(self, message):
        self.score = message

    def get_joint_states(self):
        return self.joint_states

    def get_joint_pos(self, joint_name):
        if joint_name in self.joint_states.name:
            return self.joint_states.position[self.joint_states.name.index(joint_name)]
        rospy.logerr("get_joint_state: unknown joint:" + str(joint_name))
        rospy.loginfo("get_joint_state: valid joints" + str(self.joint_states.name))

    @Sync(odom_lock)
    def get_odom_location(self):
        return self.OdomLocation

    @Sync(odom_lock)
    def get_true_pose(self):
        if self.truePoseCalled:
            rospy.logwarn("True pose already called once.")
            # @TODO if the rover has moved 2m+ might be more useful to apply the offset to odom and return that
            # as this assumes the rover has not moved since the prior call
            return self.true_pose_got
        else:
            try:
                self.truePoseCalled = True
                self.true_pose_got = self.localization_service(call=True).pose
                rospy.logwarn("true_pose_got:")
                rospy.logwarn(self.true_pose_got.position)
                odom_p = self.OdomLocation.Odometry.pose.pose.position
                odom_o = self.OdomLocation.Odometry.pose.pose.orientation
                true_pos = self.true_pose_got  # Pose
                true_p = true_pos.position  # Point
                true_o = true_pos.orientation  # Quaternion
                self.world_offset = Pose(Point(true_p.x - odom_p.x, true_p.y - odom_p.y, true_p.z - odom_p.z),
                                         Quaternion(true_o.x - odom_o.x, true_o.y - odom_o.y, true_o.z - odom_o.z,
                                                    odom_o.w - true_o.w))

                return self.true_pose_got  # @TODO might save this as a rosparam so if scoot crashes
            except (rospy.ServiceException, AttributeError) as exc:
                rospy.logwarn("Service did not process request: " + str(exc))

    @Sync(health_lock)
    def get_power_level(self):
        if self.health:
            return self.health.power_level
        rospy.logerr("No rover system_monitor messages received " +
                     "please check system_monitor_enabled is set to true in the yaml")

    @Sync(health_lock)
    def is_solar_charging(self):
        if self.health:
            return self.health.solar_ok
        rospy.logerr("No rover system_monitor messages received " +
                     "please check system_monitor_enabled is set to true in the yaml")

    def _power_saver(self, enabled):
        self.power_saver_state = enabled
        try:
            result = self.power_saver_service(power_save=enabled)
            rospy.loginfo(result.message)
            return result.success
        except (rospy.ServiceException, AttributeError) as exc:
            rospy.logwarn("power_saver_enable service did not process request: " + str(exc))

    def power_saver_on(self):
        self._power_saver(True)

    def power_saver_off(self):
        self._power_saver(False)

    def transform_pose(self, target_frame, pose, timeout=3.0):
        """Transform PoseStamped into the target frame of reference.
        Returns a PoseStamped in the target frame.

        Args:
        * target_frame (`string`) - the frame of reference to transform to. Ex: '/odom' or `/base_link`
        * pose (`PoseStamped`) - the pose of the tag in the /camera_link frame
        * timeout (`float`) - the time to wait for the transform

        Returns:
        * pose - PoseStamped the pose of the tag in the /odom frame

        Raises:
        * tf.Exception if timeout is exceeded
        """
        target_frame = self.rover_name + '_tf/' + target_frame.strip('/')

        self.xform.waitForTransform(
            pose.header.frame_id,
            target_frame,
            pose.header.stamp,
            rospy.Duration(int(timeout))
        )

        return self.xform.transformPose(target_frame, pose)

    def get_control_data(self):
        return self.control_data

    def get_volatile_pose(self):
        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.frame_id = '/small_scout_1_tf/chassis'
        pose_stamped.header.stamp = rospy.Time.now()
        odom_p = self.OdomLocation.Odometry.pose.pose.position
        odom_o = self.OdomLocation.Odometry.pose.pose.orientation
        self.get_true_pose()
        offset_pos = self.world_offset.position  # Point
        offset_ori = self.world_offset.orientation  # Quaternion
        pose_stamped.pose.pose.position = Point(odom_p.x + offset_pos.x, odom_p.y + offset_pos.y,
                                                odom_p.z + offset_pos.z)
        pose_stamped.pose.pose.orientation = Quaternion(odom_o.x + offset_ori.x, odom_o.y + offset_ori.y,
                                                        odom_o.z + offset_ori.z, odom_o.w + offset_ori.w)
        ps = PoseStamped()
        ps.header.frame_id = pose_stamped.header.frame_id
        ps.header.stamp = pose_stamped.header.stamp
        quat = [pose_stamped.pose.pose.orientation.x,
                pose_stamped.pose.pose.orientation.y,
                pose_stamped.pose.pose.orientation.z,
                pose_stamped.pose.pose.orientation.w,
                ]
        (r, p, theta) = tf.transformations.euler_from_quaternion(quat)
        add_x = math.cos(theta) * 1.143
        add_y = math.sin(theta) * 1.143
        ps.pose.position.x = pose_stamped.pose.pose.position.x + add_x
        ps.pose.position.y = pose_stamped.pose.pose.position.y - add_y
        ps.pose.position.z = 0.0

        return ps.pose.position

    # forward offset allows us to have a fixed additional distance to drive. Can be negative to underdrive to a
    # location. Motivated by the claw extension.
    def drive_to(self, place, forward_offset=0, **kwargs):
        """Drive directly to a particular point in space. The point must be in
        the odometry reference frame.

        Arguments:

        * `place`: (`geometry_msgs.msg.Point` or `geometry_msgs.msg.Pose2D`): The place to drive.
        # This is OK because they have the same member variables.
        # Being agnostic about the type allows us to handle Pose2D and Point messages with the same code.
        # Actually just requires that the object has TODO member variables and methods.

        Keyword Arguments/Returns/Raises:

        * See `scoot.scoot.drive`
        * forward_offset to the odometry reference frame.  Appropriate value
        to be passed in, otherwise the reference frame remains unchanged.

        """
        loc = self.get_odom_location().get_pose()
        dist = math.hypot(loc.y - place.y, loc.x - place.x)
        angle = angles.shortest_angular_distance(loc.theta,
                                                 math.atan2(place.y - loc.y,
                                                            place.x - loc.x))
        effective_dist = dist - forward_offset

        if effective_dist < 0:
            # The driver API skips the turn state if the request distance is
            # negative. This ensures the rover will perform the turn before
            # backing up slightly in this case.
            self.turn(angle, **kwargs)
            return self.drive(effective_dist, **kwargs)

        req = MoveRequest(
            theta=angle,
            r=effective_dist,
        )
        return self.__drive(req, **kwargs)

    def set_heading(self, heading, **kwargs):
        """Turn to face an absolute heading in radians. (zero is east)
        Arguments:
        * `heading`: (`float`) The heading in radians.
        """
        loc = self.get_odom_location().get_pose()
        angle = angles.shortest_angular_distance(loc.theta, heading)
        self.turn(angle, **kwargs)

    @Sync(drive_lock)
    def __drive(self, request, **kwargs):
        request.obstacles = ~0
        if 'ignore' in kwargs:
            request.obstacles = ~kwargs['ignore']
            '''
            if kwargs['ignore'] & Obstacle.INSIDE_HOME == Obstacle.INSIDE_HOME:
                rospy.logwarn_throttle(10.0,
                                       'Ignoring INSIDE_HOME exceptions.')
            if kwargs['ignore'] & Obstacle.VISION_HOME == Obstacle.TAG_HOME:
                rospy.logwarn_throttle(
                    10.0,
                    'Ignoring only TAG_HOME and not also HOME_CORNER. ' +
                    'You usually want to use ignore=VISION_HOME'
                )
            '''
        request.timeout = 120  # In seconds
        if 'timeout' in kwargs:
            request.timeout = kwargs['timeout']

        if 'linear' in kwargs:
            request.linear = kwargs['linear']

        if 'angular' in kwargs:
            request.angular = kwargs['angular']

        move_result = self.control([request]).result
        value = move_result.result
        obstacle = move_result.obstacle
        data = move_result.obstacle_data
        distance = move_result.distance
        heading = move_result.heading

        # Always raise AbortExceptions when the service response is USER_ABORT,
        # even if throw=False was passed as a keyword argument.
        if value == MoveResult.USER_ABORT:
            raise AbortException(value)

        if 'throw' not in kwargs or kwargs['throw']:
            if value == MoveResult.OBSTACLE_LASER:
                self.dist_data = distance
                raise ObstacleException(value, obstacle, distance)
            elif value == MoveResult.OBSTACLE_VOLATILE:
                self.control_data = data  # behaviors would fetch
                raise VolatileException(value)
            elif value == MoveResult.TIMEOUT:
                raise TimeoutException(value)
            elif value == MoveResult.VISION_VOLATILE:
                raise VisionVolatileException(heading, distance)
            elif value == MoveResult.CUBESAT:
                self.cubesat_point = rospy.get_param("/" + self.rover_name + "/cubesat_point_from_rover",
                                                     default={'x': 0, 'y': 0, 'z': 0})
                self.cubesat_point = Point(self.cubesat_point.x, self.cubesat_point.y, self.cubesat_point.z)
                raise CubesatException(heading, distance, self.cubesat_point)
            elif value == MoveResult.HOME_LEG:
                raise HomeLegException(heading, distance)
            elif value == MoveResult.HOME_FIDUCIAL:
                raise HomeLogoException(heading, distance)
        return value

    def drive(self, distance, **kwargs):
        req = MoveRequest(
            x=distance,
        )
        return self.__drive(req, **kwargs)

    def translate(self, x, y, **kwargs):
        req = MoveRequest(
            x=x,
            y=y
        )
        return self.__drive(req, **kwargs)

    def turn(self, theta, **kwargs):
        if abs(theta) < math.pi / 16:
            return
        req = MoveRequest(
            theta=theta,
        )
        return self.__drive(req, **kwargs)

    def timed_drive(self, time, linear, angular, **kwargs):
        req = MoveRequest(
            timer=time,
            linear=linear,
            angular=angular,
        )
        return self.__drive(req, **kwargs)

    @Sync(joint_lock)
    def _look(self, pitch=0.0, yaw=0.0):
        if pitch < -math.pi / 3:
            pitch = -math.pi / 3
        elif pitch > math.pi / 3:
            pitch = math.pi / 3
        if yaw < -math.pi:
            yaw = -math.pi
        elif yaw > math.pi:
            yaw = math.pi
        self.sensor_pitch_control_topic.publish(pitch)  # Up and Down
        self.sensor_yaw_control_topic.publish(yaw)  # Left and Right

    def look_up(self):
        self._look(-math.pi / 8.0)

    def look_forward(self):
        self._look(0, 0)

    def look_down(self):
        self._look(math.pi / 4.0)

    def look_right(self):
        self._look(0, -math.pi / 2)

    def look_left(self):
        self._look(0, math.pi / 2)

    def look_back(self):
        self._look(0, math.pi)

    # # # EXCAVATOR SPECIFIC CODE # # #
    def bucket_info(self):
        if self.rover_type != "excavator":
            rospy.logerr("bucket_info:" + self.rover_type + " is not an excavator")
        return self.bucket_info_msg  # last message from the bucket_info topic bucket_info srcp2_msgs/ExcavatorMsg

    @Sync(joint_lock)
    def move_shoulder_yaw(self, angle):
        """ shoulder_yaw "#1" has full horizontal rotation motion
        Allows the rover "excavator" move volatiles between volatile and hauler without needing to move the wheels
        @NOTE: Effort Limits are ignored
        """
        if self.rover_type != "excavator":
            rospy.logerr("move_shoulder_yaw:" + self.rover_type + " is not an excavator")
            return
        if angle > math.pi:
            rospy.logerr("move_shoulder_yaw:" + str(angle) + " exceeds allowed limits moving to max position")
            angle = math.pi  # max
        elif angle < -math.pi:
            rospy.logerr("move_shoulder_yaw:" + str(angle) + " exceeds allowed limits moving to minimum position")
            angle = -math.pi  # min
        self.shoulder_yaw_control.publish(angle)  # publishes angle on the shoulder_yaw_joint_controller/command topic

    @Sync(joint_lock)
    def move_shoulder_pitch(self, angle):
        """ Base Arm "#2" limited vertical motion -math.pi/5 to math.pi/3 radians
        Best bang for our buck, in regards to arm movement as its the biggest part
        Good for reaching
        @NOTE: Effort Limits are ignored
        """
        if self.rover_type != "excavator":
            rospy.logerr("move_shoulder_pitch:" + self.rover_type + " is not an excavator")
            return
        # checking bounds
        if angle > (3 * math.pi / 8.0):
            rospy.logerr("move_shoulder_pitch:" + str(angle) + " exceeds allowed limits moving to max position")
            angle = 3 * math.pi / 8.0  # max
        elif angle < (-3 * math.pi / 8.0):
            rospy.logerr("move_shoulder_pitch:" + str(angle) + " exceeds allowed limits moving to minimum position")
            angle = -3 * math.pi / 8.0  # min
        self.shoulder_pitch_control.publish(angle)

    @Sync(joint_lock)
    def move_elbow_pitch(self, angle):
        """Distal Arm "#3" limited vertical motion -math.pi/3 to math.pi/3 radians
        Good for lowering the bucket
        @NOTE: Effort Limits are ignored
        """
        if self.rover_type != "excavator":
            rospy.logerr("move_elbow_pitch:" + self.rover_type + " is not an excavator")
            return
        # checking bounds
        if angle > (7 * math.pi / 8.0):  # @NOTE: or its elbow_pitch_joint: 7pi/8 != pi/4 as the wiki says
            rospy.logerr("move_shoulder_pitch:" + str(angle) + " exceeds allowed limits moving to max position")
            angle = 7 * math.pi / 8.0  # max
        elif angle < (-math.pi / 4.0):
            rospy.logerr("move_shoulder_pitch:" + str(angle) + " exceeds allowed limits moving to minimum position")
            angle = -math.pi / 4.0  # min
        self.elbow_pitch_control.publish(angle)

    @Sync(joint_lock)
    def move_bucket(self, angle):
        # checking bounds
        if self.rover_type == "excavator":
            if angle > (2 * math.pi / 3.0):
                rospy.logerr("move_bucket:" + str(angle) + " exceeds allowed limits moving to max position")
                angle = 2 * math.pi / 3.0  # max
            elif angle < (-2 * math.pi / 3.0):
                rospy.logerr("move_bucket:" + str(angle) + " exceeds allowed limits moving to minimum position")
                angle = -2 * math.pi / 3.0  # min
            self.bucket_control.publish(angle)
            return
        rospy.logerr("move_bucket:" + self.rover_type + " is not an excavator")

    def get_shoulder_yaw_angle(self):
        if self.rover_type != "excavator":
            rospy.logerr("get_shoulder_yaw_angle:" + self.rover_type + " is not an excavator")
            return
        return self.get_joint_pos("shoulder_yaw_joint")

    def get_shoulder_pitch_angle(self):
        if self.rover_type != "excavator":
            rospy.logerr("get_shoulder_pitch_angle:" + self.rover_type + " is not an excavator")
            return
        return self.get_joint_pos("basearm_joint")

    def get_elbow_pitch_angle(self):
        if self.rover_type != "excavator":
            rospy.logerr("get_elbow_pitch_angle:" + self.rover_type + " is not an excavator")
            return
        return self.get_joint_pos("distalarm_joint")

    def get_bucket_angle(self):
        if self.rover_type != "excavator":
            rospy.logerr("get_bucket_angle:" + self.rover_type + " is not an excavator")
            return
        return self.get_joint_pos("bucket_joint")

    # # # END EXCAVATOR SPECIFIC CODE # # #

    # # # HAULER SPECIFIC CODE # # #
    def bin_info(self):
        if self.rover_type != "hauler":
            rospy.logerr("bin_info:" + self.rover_type + " is not a hauler")
            return
        return self.score.hauler_volatile_score

    @Sync(joint_lock)
    def move_bin(self, angle):
        if self.rover_type != "hauler":
            rospy.logerr("move_bin:" + self.rover_type + " is not a hauler")
            return
        if angle < 0:
            angle = 0
        elif angle > (3 * math.pi / 4):
            angle = (3 * math.pi / 4)
        self.bin_control.publish(angle)

    def bin_home(self):
        self.move_bin(0)

    def bin_dump(self):
        self.move_bin(3 * math.pi / 4)

    def get_bin_angle(self):
        if self.rover_type != "hauler":
            rospy.logerr("get_bin_angle:" + self.rover_type + " is not a hauler")
            return
        return self.get_joint_pos("bin_joint")

    # # # END HAULER SPECIFIC CODE # # #

    def get_next_best_vol_pose(self):
        # @SEE https://gitlab.com/scheducation/srcp2-final-public/-/wikis/1.-General/3.-Scoring-and-Objectives
        pass  # @TODO Min Quantity and Priority Queue
        rover_pose = self.get_odom_location().get_pose()
        """
        Look at /srcp2/score types_collected & masses_collected_kg list zip them
        Minimum Required Quantity:
        ice 60
        carbon_dioxide 3
        ammonia 4
        hydrogen_sulfite 10
        sulfur_dioxide 2
        
        Once Achieved priority queue score and distance weights:
        ethane, methanol, methane
        carbon_dioxide, ammonia
        hydrogen_sulfite, sulfur_dioxide, ice
        """

    def get_closest_vol_pose(self):
        try:
            while rospy.get_param("/volatile_locations_latch", default=False):
                rospy.sleep(0.2)  # wait for it be be unlatched
            rospy.set_param('/volatile_locations_latch', True)  # this is to support multiple rovers
            volatile_locations = rospy.get_param("/volatile_locations", default=list())
        finally:
            rospy.set_param('/volatile_locations_latch', False)
        if not volatile_locations:  # No volatiles, behavior should then wait for non None return
            return None
        rover_pose = self.get_odom_location().get_pose()

        closest_vol_pose = min(volatile_locations,
                               key=lambda k: math.sqrt((k['x'] - rover_pose.x) ** 2 + (k['y'] - rover_pose.y) ** 2))
        rospy.loginfo("rover pose:           x:" + str(rover_pose.x) + ", y:" + str(rover_pose.y))
        rospy.loginfo("get_closest_vol_pose: x:" + str(closest_vol_pose['x']) + ", y:" + str(closest_vol_pose['y']))
        return closest_vol_pose
        # If we wanted to return all the elements we would get the index from min or find the index
        # where the min pose is at
        # (vol_list.poses[index], vol_list.is_shadowed[index], vol_list.starting_mass[index],
        # vol_list.volatile_type[index])

    def remove_closest_vol_pose(self):
        try:
            while rospy.get_param("/volatile_locations_latch", default=False):
                rospy.sleep(0.2)  # wait for it be be unlatched
            rospy.set_param('/volatile_locations_latch', True)  # this is to support multiple rovers
            volatile_locations = rospy.get_param("/volatile_locations", default=list())
            rover_pose = self.get_odom_location().get_pose()
            closest_vol_pose = min(volatile_locations,
                                   key=lambda k: math.sqrt((k['x'] - rover_pose.x) ** 2 + (k['y'] - rover_pose.y) ** 2))
            volatile_locations.remove(closest_vol_pose)
            rospy.set_param('/volatile_locations', volatile_locations)
        finally:
            rospy.set_param('/volatile_locations_latch', False)
