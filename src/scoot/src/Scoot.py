#!/usr/bin/env python
import rospy
import math
import angles 
import tf
import threading
import numpy

from rospy import ServiceException
from srcp2_msgs import msg, srv

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose2D, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from scoot.msg import MoveResult, MoveRequest

from gazebo_msgs.srv import GetModelState
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
    pass


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

        self.TURN_SPEED = 0
        self.DRIVE_SPEED = 0
        self.REVERSE_SPEED = 0
        self.MAX_BRAKES = 0

        self.skidTopic = None
        self.sensorControlTopic = None

        self.lightService = None
        self.brake_service = None
        self.localizationService = None
        self.modelStateService = None

        self.truePoseCalled = False

        self.OdomLocation = Location(None)
        self.control = None
        self.control_data = None
        self.xform = None

    def start(self, **kwargs):
        """
        if 'tf_rover_name' in kwargs :
            self.rover_name = kwargs['tf_rover_name']
        else:
            self.rover_name = rospy.get_namespace()
        self.rover_name = self.rover_name.strip('/')
        """
        self.rover_name = rospy.get_param('rover_name', default='scout_1')
        self.rover_type = rospy.get_param('rover_type', default='scout')
        self.TURN_SPEED = rospy.get_param("TURN_SPEED", default=0.6)
        self.DRIVE_SPEED = rospy.get_param("DRIVE_SPEED", default=0.3)
        self.REVERSE_SPEED = rospy.get_param("REVERSE_SPEED", default=0.2)
        self.MAX_BRAKES = rospy.get_param("MAX_BRAKES", default=499)

        '''Tracking SRCP2's Wiki 
                Documentation/API/Robots/Hauler.md  
                Documentation/Qualification-Rounds/Round-Two.md 
                Documentation/Qualification-Rounds/Round-Two.md 
            HaulerMsg.msg's comment in srcp2-competitors/ros_workspace/install/shares/rcp2_msgs/msg/
            our scoot.launch
            and matching with indexes from our Obstacles.msg '''
        self.VOL_TYPES = rospy.get_param("vol_types",
                                         default=["ice", "ethene", "methane", "methanol", "carbon_dio", "ammonia",
                                                  "hydrogen_sul", "sulfur_dio"])

        #  @NOTE: when we use namespaces we wont need to have the rover_name
        # Create publishers.
        self.skidTopic = rospy.Publisher('/' + self.rover_name + '/skid_cmd_vel', Twist, queue_size=10)
        self.sensorControlTopic = rospy.Publisher('/' + self.rover_name + '/sensor_controller/command', Float64,
                                                  queue_size=10)

        # Connect to services.
        rospy.loginfo("Waiting for control service")
        rospy.wait_for_service('/' + self.rover_name + '/control')
        self.control = rospy.ServiceProxy('/' + self.rover_name + '/control', Core)
        rospy.loginfo("Done waiting for control service")

        rospy.wait_for_service('/' + self.rover_name + '/toggle_light')
        self.lightService = rospy.ServiceProxy('/' + self.rover_name + '/toggle_light', srv.ToggleLightSrv)
        rospy.wait_for_service('/' + self.rover_name + '/brake_rover')
        self.brake_service = rospy.ServiceProxy('/' + self.rover_name + '/brake_rover', srv.BrakeRoverSrv)
        rospy.wait_for_service('/' + self.rover_name + '/get_true_pose')
        self.localizationService = rospy.ServiceProxy('/' + self.rover_name + '/get_true_pose', srv.LocalizationSrv)
        rospy.wait_for_service('/vol_detected_service')
        self.qal1ScoreService = rospy.ServiceProxy('/vol_detected_service', srv.Qual1ScoreSrv)

        # Subscribe to topics.
        rospy.Subscriber('/' + self.rover_name + '/odom/filtered', Odometry, self._odom)
        
        # Transform listener. Use this to transform between coordinate spaces.
        # Transform messages must predate any sensor messages so initialize this first.
        self.xform = tf.TransformListener()
        rospy.loginfo("Scoot Ready")

    # @sync(odom_lock)
    def _odom(self, msg):
        self.OdomLocation.Odometry = msg

    def getOdomLocation(self):
        with odom_lock:
            return self.OdomLocation

    def getTruePose(self):
        if self.truePoseCalled:
            print("True pose already called once.")
            return
        else:
            try:
                l = self.localizationService(call=True)
                quat = [l.pose.orientation.x,
                        l.pose.orientation.y,
                        l.pose.orientation.z,
                        l.pose.orientation.w,
                        ]
                (r, p, y) = tf.transformations.euler_from_quaternion(quat)

                pose = Pose2D()
                pose.x = l.pose.position.x
                pose.y = l.pose.position.y
                pose.theta = y

                self.truePoseCalled = True

                return pose
            except rospy.ServiceException as exc:
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
            target_frame,
            pose.header.frame_id,
            pose.header.stamp,
            rospy.Duration(timeout)
        )

        return self.xform.transformPose(target_frame, pose)

    def getControlData(self):
        return self.control_data

    # @TODO: test this
    def score(self, vol_type_index=0):
        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.frame_id = '/scout_1_tf/chassis'
        pose_stamped.header.stamp = rospy.Time.now() 
        pose_stamped.pose = self.OdomLocation.Odometry.pose.pose
        aprox_vol_location = self.transform_pose("volatile_sensor_static", pose_stamped)
        result = None
        try:
            result = self.qal1ScoreService(
                pose=aprox_vol_location.pose.position, 
                vol_type=self.VOL_TYPES[vol_type_index])
            rospy.loginfo("Scored!")
        except ServiceException:
            rospy.logwarn("/vol_detected_service is grumpy")
            result = False
        return result
      
    # forward offset allows us to have a fixed addional distance to drive. Can be negative to underdrive to a location. Motivated by the claw extention. 
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
        request.timeout = 120 # In seconds
        if 'timeout' in kwargs:
            request.timeout = kwargs['timeout']

        if 'linear' in kwargs:
            request.linear = kwargs['linear']

        if 'angular' in kwargs:
            request.angular = kwargs['angular']

        move_result = self.control([request]).result
        value = move_result.result
        data = move_result.obstacle_data

        # Always raise AbortExceptions when the service response is USER_ABORT,
        # even if throw=False was passed as a keyword argument.
        if value == MoveResult.USER_ABORT:
            raise AbortException(value)

        if 'throw' not in kwargs or kwargs['throw']:
            if value == MoveResult.OBSTACLE_LASER:
                raise ObstacleException(value)
            elif value == MoveResult.OBSTACLE_VOLATILE:
                self.control_data = data  # behaviors would fetch and call score
                raise VolatileException(value)
            elif value == MoveResult.TIMEOUT:
                raise TimeoutException(value)
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

    def _brake_ramp(self, end_brake_value=499, stages=10, hz=10, exponent=1.3):
        """
        Applies the brakes "gradually"
        Given the defaults it will apply the below values to the brakes over the course of 1 second
        [1, 1, 3, 7, 15, 31, 62, 125, 250, 499]
        end_brake_value: is just that it it the final value that will be sent to the brake service
        stages: is the number of distinct values that will be sent to the brake service
        hz: is number of states that happen per a second
        exponent: is a magic number that will control the brake value ramping
        """
        rate = rospy.Rate(hz)  # default 10hz
        for brake_value in list(numpy.logspace(0, math.log(end_brake_value, exponent), base=exponent, dtype='int',
                                               endpoint=True, num=stages)):
            self.brake_service.call(brake_value)
            rate.sleep()

    def brake(self, state=None):
        if (state == "on") or (state is True) or (state is None):
            self._brake_ramp(self.MAX_BRAKES)
        elif (state == "off") or (state is False) or (state == 0.0):
            self.brake_service(0)  # immediately disengage brakes
        elif (type(state) != float) and (type(state) != int):
            rospy.logerr("Invalid brake value, got:" + str(state))
        elif state < 0:
            rospy.logerr("Brake value can't be negative, got:" + str(state))
            rospy.logwarn("Disengaging brakes")
            self.brake_service(0)  # immediately disengage brakes
        elif state >= (self.MAX_BRAKES + 1):
            rospy.logerr("Brake value can't greater/equal to "+str(self.MAX_BRAKES)+", got:" + str(state))
            rospy.logwarn("Applying full brakes")
            self._brake_ramp(self.MAX_BRAKES)
        else:
            self._brake_ramp(state)

    def _light(self, state):
        self.lightService(data=state)

    def lightOn(self):
        self._light('high')

    def lightLow(self):
        self._light('low')

    def lightOff(self):
        self._light('stop')

    def _look(self, angle):
        self.sensorControlTopic.publish(angle)

    def lookUp(self):
        self._look(math.pi / 4.0)

    def lookForward(self):
        self._look(0)

    def lookDown(self):
        self._look(-math.pi / 8.0)

