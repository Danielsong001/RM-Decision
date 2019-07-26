#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node('A_poseupdate')
    mypose_pub = rospy.Publisher('robot_1/my_pose', Odometry, queue_size=1)
    tflistener = tf.TransformListener()
    pose = Odometry()
    pose.header.frame_id = 'map'
    pose.child_frame_id = 'robot_1/base_link'

    rate = rospy.Rate(40.0)
    while not rospy.is_shutdown():
        try:
            t, q = tflistener.lookupTransform("/map", "robot_1/base_link", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        pose.pose.pose.position.x = t[0]
        pose.pose.pose.position.y = t[1]
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        mypose_pub.publish(pose)
        rate.sleep()
        print 'update robot1'
