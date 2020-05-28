import rospy
import math
import tf
from std_msgs.msg import String, Float64
from srcp2_msgs import msg, srv
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
import threading

swarmie_lock = threading.Lock()

class Location:
    def __init__(self, odo):
        self.Odometry = odo

    def getPose(self):
        quat = [self.Odometry.pose.pose.orientation.x,
                self.Odometry.pose.pose.orientation.y,
                self.Odometry.pose.pose.orientation.z,
                self.Odometry.pose.pose.orientation.w,
                ]
        (r, p, y) = tf.transformations.euler_from_quaternion(quat)

        pose = Pose2D()
        pose.x = self.Odometry.pose.pose.position.x
        pose.y = self.Odometry.pose.pose.position.y
        pose.theta = y

        return pose

    def atGoal(self, goal, distance):
        dist = math.hypot(goal.x - self.Odometry.pose.pose.position.x,
                          goal.y - self.Odometry.pose.pose.position.y)

        return dist < distance

class Scoot(object):
    def __init__(self, rover):
        self.rover_name = None
        
        self.TURN_SPEED = 0
        self.DRIVE_SPEED = 0
        self.REVERSE_SPEED = 0

        self.skidTopic = None
        self.sensorControllTopic = None

        self.lightService = None
        self.breaksService = None
        self.localizationService = None
        self.modelStateService = None

        self.truePoseCalled = False

        self.odomLocation = Location(None)
    
    def start(self):
        '''
        if 'tf_rover_name' in kwargs :
            self.rover_name = kwargs['tf_rover_name']
        else:
            self.rover_name = rospy.get_namespace()
        self.rover_name = self.rover_name.strip('/')
        '''
        self.rover_name = "scout_1"
        self.TURN_SPEED = rospy.get_param("TURN_SPEED", default=0.6)
        self.DRIVE_SPEED = rospy.get_param("DRIVE_SPEED", default=0.3)
        self.REVERSE_SPEED = rospy.get_param("REVERSE_SPEED", default=0.2)
        
        #  @NOTE: when we use namespaces we wont need to have the rover_name
        self.skidTopic = rospy.Publisher('/'+self.rover_name+'/skid_cmd_vel', Twist, queue_size=10)
        self.sensorControllTopic = rospy.Publisher('/'+self.rover_name+'/sensor_controller/command', Float64, queue_size=10)

        rospy.wait_for_service('/'+self.rover_name+'/toggle_light')
        self.lightService = rospy.ServiceProxy('/'+self.rover_name+'/toggle_light', srv.ToggleLightSrv)
        rospy.wait_for_service('/'+self.rover_name+'/brake_rover')
        self.breaksService = rospy.ServiceProxy('/'+self.rover_name+'/brake_rover', srv.BrakeRoverSrv)
        rospy.wait_for_service('/'+self.rover_name+'/get_true_pose')
        self.localizationService = rospy.ServiceProxy('/'+self.rover_name+'/get_true_pose', srv.LocalizationSrv)

        rospy.Subscriber('/'+self.rover_name+'/odom/filtered', Odometry, self._odom)
        
    def _drive(self, linear, angular, mode):
        t = Twist() 
        t.linear.x = linear
        t.angular.y = mode
        t.angular.z = angular
        self.skidTopic.publish(t) 
        
    def drive(self, speed=5):  # @TODO: when we have odom change drive/pirouette speeds to distance/angle
        self._drive(speed,0,0)
        
    def pirouette(self, speed=5):
        self._drive(0,speed,0)
        
    def stop(self):
        self._drive(0,0,0)
        
    def breaks(self, state="on"):
        self.breaksService(state is "on")

    def _light(self, state):
        self.lightService(data=state)

    def lightOn(self):
        self._light('high')

    def lightLow(self):
        self._light('low')

    def lightOff(self):
        self._light('stop')
        
    def _look(self, angle):
        self.sensorControllTopic.publish(angle)
    
    def lookUp(self):
        self._look(math.pi/4.0)
        
    def lookForward(self):
        self._look(0)
    
    def lookDown(self):
        self._look(-math.pi/8.0)

    def getTruePose(self):
        if not self.truePoseCalled:
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

    def _odom(self, msg):
        self.odomLocation.Odometry = msg

    def getOdomLocation(self):
        with swarmie_lock:
            return self.odomLocation
    
if __name__ == "__main__": 
    rospy.init_node('ScootNode')
    scoot = Scoot("scout_1")
    #rospy.spin()
    #Systems will have an unmet dependency run "sudo pip install ipython"
    try :
        from IPython import embed
        embed(user_ns=globals())
    except ImportError as e:
        print("Missing IPython run 'sudo pip install ipython'\n Failing over")
        try: 
            while True : 
                line = raw_input('>>> ')
                if line is not None and line != '' :
                    try :
                        exec (line)
                    except Exception as e :
                        print (e)
        except EOFError as e : 
            print ("Goodbye")
    print ("Qapla'!")
       
