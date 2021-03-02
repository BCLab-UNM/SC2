#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState


def publish_fake_odom():
    rover_name = rospy.get_param('rover_name', default='small_scout_1')
    pub = rospy.Publisher('/' + rover_name + '/odometry/filtered', Odometry, queue_size=10)
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
        pose.header.seq = m.header.seq
        pose.header.stamp = m.header.stamp
        pose.header.frame_id = m.header.frame_id

        pose.pose.pose.orientation.x = m.pose.orientation.x
        pose.pose.pose.orientation.y = m.pose.orientation.y
        pose.pose.pose.orientation.z = m.pose.orientation.z
        pose.pose.pose.orientation.w = m.pose.orientation.w

        pose.pose.pose.position.x = m.pose.position.x
        pose.pose.pose.position.y = m.pose.position.y
        pose.pose.pose.position.z = m.pose.position.z

        pose.twist.twist.linear.x = m.twist.linear.x
        pose.twist.twist.linear.y = m.twist.linear.y
        pose.twist.twist.linear.z = m.twist.linear.z

        pose.twist.twist.angular.x = m.twist.angular.x
        pose.twist.twist.angular.y = m.twist.angular.y
        pose.twist.twist.angular.z = m.twist.angular.z

        pub.publish(pose)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_fake_odom()
    except rospy.ROSInterruptException:
        pass
