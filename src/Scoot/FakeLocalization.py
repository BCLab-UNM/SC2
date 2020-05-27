import rospy
from geometry_msgs.msg import Pose2D
from Scoot import *

def publishFakeOdom(rover):
    pub = rospy.Publisher('/'+rover.rover_name+'/fakeOdom', Pose2D, queue_size=10)
    rospy.init_node('odom', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m = rover.getModelState()
        quat = [m.pose.orientation.x,
                m.pose.orientation.y,
                m.pose.orientation.z,
                m.pose.orientation.w,
        ]
        (r, p, y) = tf.transformations.euler_from_quaternion(quat)

        pose = Pose2D()
        pose.x = m.pose.position.x
        pose.y = m.pose.position.y
        pose.theta = y

        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        scoot = Scoot("scout_1")
        scoot.start()
        publishFakeOdom(scoot)
    except rospy.ROSInterruptException:
        pass
        
