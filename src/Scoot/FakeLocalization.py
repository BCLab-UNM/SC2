import rospy
import tf
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState

def publishFakeOdom():
    rover_name = rospy.get_param('rover_name', default='scout_1')
    pub = rospy.Publisher('/'+rover_name+'/odom/filtered', Odometry, queue_size=10)
    rospy.init_node('odom', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        m = model_coordinates(rover_name, 'world')
        quat = [m.pose.orientation.x,
                m.pose.orientation.y,
                m.pose.orientation.z,
                m.pose.orientation.w,
        ]
        (r, p, y) = tf.transformations.euler_from_quaternion(quat)

        pose = Odometry()
        pose.pose.pose.orientation.x = m.pose.orientation.x
        pose.pose.pose.orientation.y = m.pose.orientation.y
        pose.pose.pose.orientation.z = m.pose.orientation.z
        pose.pose.pose.orientation.w = m.pose.orientation.w
        
        pose.pose.pose.position.x = m.pose.position.x
        pose.pose.pose.position.y = m.pose.position.y
        pose.pose.pose.position.z = m.pose.position.z

        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        publishFakeOdom()
    except rospy.ROSInterruptException:
        pass
        
