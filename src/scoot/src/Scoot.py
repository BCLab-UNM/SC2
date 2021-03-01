#!/usr/bin/env python3
import rospy
import math
import angles
import tf
import threading
import numpy

from rospy import ServiceException
from srcp2_msgs import msg, srv

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose2D, Point, PoseWithCovariance, PoseWithCovarianceStamped, PoseStamped, \
    Quaternion, Pose
from nav_msgs.msg import Odometry
from scoot.msg import MoveResult, MoveRequest
from scoot.srv import Core

from functools import wraps

odom_lock = threading.Lock()


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


# def sync(lock):
#         def _sync(func):
#             @wraps(func)
#             def wrapper(*args, **kwargs):
#                 with lock:
#                     return func(*args, **kwargs)
#                 return wrapper
#         return _sync

class Location:
    """A class that encodes an EKF provided location and accessor methods"""

    def __init__(self, odo):
        self.Odometry = odo

    def getPose(self):  # TODO: add a ros warning if self.Odometry none
        '''Return a std_msgs.Pose from this Location. Useful because Pose 
        has angles represented as roll, pitch, yaw.
            
        Returns:
            
        * (`std_msgs.msg.Pose`) The pose. 
        '''
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

    def atGoal(self, goal, distance):
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
        self.rover_name = None
        self.rover_type = None
        self.TURN_SPEED = 0
        self.DRIVE_SPEED = 0
        self.REVERSE_SPEED = 0
        self.VOL_TYPES = None
        self.ROUND_NUMBER = 0

        self.skid_topic = None
        self.sensor_control_topic = None
        self.mount_control = None
        self.base_arm_control = None
        self.bucket_control = None
        self.distal_arm_control = None
        self.bin_control = None

        self.bucket_info_msg = None
        self.bin_info_msg = None

        self.localization_service = None
        self.model_state_service = None
        self.vol_list_service = None

        self.truePoseCalled = False
        self.true_pose_got = None

        self.OdomLocation = Location(None)
        self.home_pose = Point()
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

    def start(self, **kwargs):
        if 'tf_rover_name' in kwargs:
            self.rover_name = kwargs['tf_rover_name']
        else:
            self.rover_name = rospy.get_namespace()
        self.rover_name = self.rover_name.strip('/')
        self.rover_type = self.rover_name.split("_")[0]  # cuts of the _# part of the rover name
        self.DRIVE_SPEED = rospy.get_param("/"+self.rover_name+"/Core/DRIVE_SPEED", default=5)
        self.REVERSE_SPEED = rospy.get_param("/"+self.rover_name+"/Core/REVERSE_SPEED", default=5)
        self.TURN_SPEED = rospy.get_param("/"+self.rover_name+"/Core/TURN_SPEED", default=5)
        self.ROUND_NUMBER = rospy.get_param('round', default=1)

        '''Tracking SRCP2's Wiki 
                Documentation/API/Robots/Hauler.md  
                Documentation/Qualification-Rounds/Round-Two.md 
                Documentation/Qualification-Rounds/Round-Two.md d
            HaulerMsg.msg's comment in srcp2-competitors/ros_workspace/install/shares/rcp2_msgs/msg/
            our scoot.launch
            and matching with indexes from our Obstacles.msg '''
        self.VOL_TYPES = rospy.get_param("vol_types",
                                         default=["ice", "ethene", "methane", "carbon_mono", "carbon_dio", "ammonia",
                                                  "hydrogen_sul", "sulfur_dio"])

        #  @NOTE: when we use namespaces we wont need to have the rover_name
        # Create publishers.
        #self.skid_topic = rospy.Publisher('/' + self.rover_name + '/skid_cmd_vel', Twist, queue_size=10)
        #self.sensor_control_topic = rospy.Publisher('/' + self.rover_name + '/sensor_controller/command', Float64,
        #                                            queue_size=10)

        # Connect to services.
        rospy.loginfo("Waiting for control service")
        rospy.wait_for_service('/' + self.rover_name + '/control')
        self.control = rospy.ServiceProxy('/' + self.rover_name + '/control', Core)
        rospy.loginfo("Done waiting for control service")

        rospy.wait_for_service('/' + self.rover_name + '/get_true_pose')
        self.localization_service = rospy.ServiceProxy('/' + self.rover_name + '/get_true_pose', srv.LocalizationSrv)
        rospy.loginfo("Done waiting for general services")
        if self.rover_type == "scout":
            if self.ROUND_NUMBER == 1:
                rospy.wait_for_service('/vol_detected_service')
        elif self.rover_type == "excavator":
            self.mount_control = rospy.Publisher('/' + self.rover_name + '/mount_joint_controller/command', Float64,
                                                 queue_size=10)
            self.base_arm_control = rospy.Publisher('/' + self.rover_name + '/basearm_joint_controller/command',
                                                    Float64, queue_size=10)
            self.bucket_control = rospy.Publisher('/' + self.rover_name + '/bucket_joint_controller/command', Float64,
                                                  queue_size=10)
            self.distal_arm_control = rospy.Publisher('/' + self.rover_name + '/distalarm_joint_controller/command',
                                                      Float64, queue_size=10)
            rospy.Subscriber('/' + self.rover_name + '/bucket_info', msg.ExcavatorMsg,
                             self._bucket_info)

        elif self.rover_type == "hauler":
            self.bin_control = rospy.Publisher('/' + self.rover_name + '/bin_joint_controller/command', Float64,
                                               queue_size=10)
            rospy.Subscriber('/' + self.rover_name + '/bin_info', msg.HaulerMsg, self._bin_info)

        rospy.loginfo("Done waiting for rover specific services")
        # Subscribe to topics.
        rospy.Subscriber('/' + self.rover_name + '/odometry/filtered', Odometry, self._odom)
        rospy.Subscriber('/' + self.rover_name + '/joint_states', JointState, self._joint_states)
        # Transform listener. Use this to transform between coordinate spaces.
        # Transform messages must predate any sensor messages so initialize this first.
        self.xform = tf.TransformListener()
        rospy.loginfo("Scoot Ready")

    # @sync(odom_lock)
    def _odom(self, msg):
        self.OdomLocation.Odometry = msg

    def _bin_info(self, msg):
        self.bin_info_msg = msg

    def _bucket_info(self, msg):
        self.bucket_info_msg = msg

    def _joint_states(self, msg):
        self.joint_states = msg

    def get_joint_states(self):
        return self.joint_states

    def get_joint_pos(self, joint_name):
        if joint_name in self.joint_states.name:
            return self.joint_states.position[self.joint_states.name.index(joint_name)]
        rospy.logerr("get_joint_state: unknown joint:" + str(joint_name))
        rospy.loginfo("get_joint_state: valid joints" + str(self.joint_states.name))

    def getOdomLocation(self):
        with odom_lock:
            return self.OdomLocation

    def getTruePose(self):
        if self.truePoseCalled:
            print("True pose already called once.")
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
                print("Service did not process request: " + str(exc))

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

    def getControlData(self):
        return self.control_data

    def getVolPose(self):
        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.frame_id = '/small_scout_1_tf/chassis'
        pose_stamped.header.stamp = rospy.Time.now()
        odom_p = self.OdomLocation.Odometry.pose.pose.position
        odom_o = self.OdomLocation.Odometry.pose.pose.orientation
        self.getTruePose()
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
        '''Drive directly to a particular point in space. The point must be in 
        the odometry reference frame. 
        
        Arguments:
        
        * `place`: (`geometry_msgs.msg.Point` or `geometry_msgs.msg.Pose2D`): The place to drive. 
        # This is OK becasue they have the same member variables.
        # Being agnostic about the type allows us to handle Pose2D and Point messages with the same code. 
        # Actually just requires that the object has TODO member variables and methods. 

        Keyword Arguments/Returns/Raises:
        
        * See `mobility.swarmie.Swarmie.drive`
        * forward_offset to the odometry reference frame.  Appropriate value
        to be passed in, otherwise the reference frame remains unchanged.
            
        '''
        loc = self.getOdomLocation().getPose()
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
        '''Turn to face an absolute heading in radians. (zero is east)
        Arguments:
        * `heading`: (`float`) The heading in radians.
        '''
        loc = self.getOdomLocation().getPose()
        angle = angles.shortest_angular_distance(loc.theta, heading)
        self.turn(angle, **kwargs)

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
                self.control_data = data  # behaviors would fetch and call score
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
            r=distance,
        )
        return self.__drive(req, **kwargs)

    def turn(self, theta, **kwargs):
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

    def _look(self, angle):
        self.sensor_control_topic.publish(angle)

    def lookUp(self):
        self._look(math.pi / 4.0)

    def lookForward(self):
        self._look(0)

    def lookDown(self):
        self._look(-math.pi / 8.0)

    # # # EXCAVATOR SPECIFIC CODE # # #
    def bucket_info(self):
        if self.rover_type != "excavator":
            rospy.logerr("bucket_info:" + self.rover_type + " is not an excavator")
        return self.bucket_info_msg  # last message from the bucket_info topic bucket_info srcp2_msgs/ExcavatorMsg

    def move_mount(self, angle):
        """ Mount "#1" has full horizontal rotation motion
        Allows the rover "excavator" move volatiles between volatile and hauler without needing to move the wheels
        @NOTE: Effort Limits are ignored
        """
        if self.rover_type != "excavator":
            rospy.logerr("move_mount:" + self.rover_type + " is not an excavator")
            return
        # @NOTE: the controller handles if values should wrap and takes the "shortest" path
        self.mount_control.publish(angle)  # publishes angle on the mount_joint_controller/command topic

    def move_base_arm(self, angle):
        """ Base Arm "#2" limited vertical motion -math.pi/5 to math.pi/3 radians
        Best bang for our buck, in regards to arm movement as its the biggest part
        Good for reaching
        @NOTE: Effort Limits are ignored
        """
        if self.rover_type != "excavator":
            rospy.logerr("move_base_arm:" + self.rover_type + " is not an excavator")
            return
        # checking bounds
        if angle > (math.pi / 3.0):
            rospy.logerr("move_base_arm:" + str(angle) + " exceeds allowed limits moving to max position")
            self.base_arm_control.publish(math.pi / 3.0)  # max
        elif angle < (-math.pi / 5.0):
            rospy.logerr("move_base_arm:" + str(angle) + " exceeds allowed limits moving to minimum position")
            self.base_arm_control.publish((-math.pi / 5.0))  # min
        else:
            self.base_arm_control.publish(angle)

    def move_distal_arm(self, angle):
        """Distal Arm "#3" limited vertical motion -math.pi/3 to math.pi/3 radians
        Good for lowering the bucket
        @NOTE: Effort Limits are ignored
        """
        if self.rover_type != "excavator":
            rospy.logerr("move_distal_arm:" + self.rover_type + " is not an excavator")
            return
        # checking bounds
        if angle > (math.pi / 3.0):
            rospy.logerr("move_base_arm:" + str(angle) + " exceeds allowed limits moving to max position")
            self.distal_arm_control.publish(math.pi / 3.0)  # max
        elif angle < (-math.pi / 3.0):
            rospy.logerr("move_base_arm:" + str(angle) + " exceeds allowed limits moving to minimum position")
            self.distal_arm_control.publish((-math.pi / 3.0))  # min
        else:
            self.distal_arm_control.publish(angle)

    def move_bucket(self, angle):
        # checking bounds
        if self.rover_type == "excavator":
            if angle > ((5 * math.pi) / 4.0):
                rospy.logerr("move_bucket:" + str(angle) + " exceeds allowed limits moving to max position")
                self.bucket_control.publish((5 * math.pi) / 4.0)  # max
            elif angle < 0:
                rospy.logerr("move_bucket:" + str(angle) + " exceeds allowed limits moving to minimum position")
                self.bucket_control.publish(0)  # min
            else:
                self.bucket_control.publish(angle)
            return
        rospy.logerr("move_bucket:" + self.rover_type + " is not an excavator")

    def get_mount_angle(self):
        if self.rover_type != "excavator":
            rospy.logerr("get_mount_angle:" + self.rover_type + " is not an excavator")
            return
        return self.get_joint_pos("mount_joint")

    def get_base_arm_angle(self):
        if self.rover_type != "excavator":
            rospy.logerr("get_base_arm_angle:" + self.rover_type + " is not an excavator")
            return
        return self.get_joint_pos("basearm_joint")

    def get_distal_arm_angle(self):
        if self.rover_type != "excavator":
            rospy.logerr("get_distal_arm_angle:" + self.rover_type + " is not an excavator")
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
        self.bin_info_msg = msg  # message from bin_info topic type srcp2_msgs/HaulerMsg

    def move_bin(self, angle):
        if self.rover_type != "hauler":
            rospy.logerr("move_bin:" + self.rover_type + " is not a hauler")
            return
        # @TODO check bounds # -math.pi \ 3 to 0
        self.bin_control.publish(angle)

    # @TODO wrappers to home and dump of bin

    def get_bin_angle(self):
        if self.rover_type != "hauler":
            rospy.logerr("get_bin_angle:" + self.rover_type + " is not a hauler")
            return
        return self.get_joint_pos("bin_joint")

    # # # END HAULER SPECIFIC CODE # # #

    def get_closest_vol_pose(self):
        rover_pose = self.getOdomLocation().getPose()
        try:
            vol_list = self.vol_list_service.call()
        except (ServiceException, AttributeError):
            rospy.logerr("get_closest_vol_pose: vol_list_service call failed")
            try:
                vol_list = self.vol_list_service.call()
                rospy.logwarn("get_closest_vol_pose: vol_list_service call succeeded")
            except (ServiceException, AttributeError):
                rospy.logerr("get_closest_vol_pose: vol_list_service call failed second time, giving up")
                return None
        closest_vol_pose = min(vol_list.poses,
                               key=lambda k: math.sqrt((k.x - rover_pose.x) ** 2 + (k.y - rover_pose.y) ** 2))
        rospy.loginfo("rover pose:           x:" + str(rover_pose.x) + ", y:" + str(rover_pose.y))
        rospy.loginfo("get_closest_vol_pose: x:" + str(closest_vol_pose.x) + ", y:" + str(closest_vol_pose.y))
        return closest_vol_pose
        # If we wanted to return all the elements we would get the index from min or find the index
        # where the min pose is at
        # (vol_list.poses[index], vol_list.is_shadowed[index], vol_list.starting_mass[index],
        # vol_list.volatile_type[index])
