#!/usr/bin/python
# -*- coding: utf-8 -*-

import copy
import random

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

from lama_msgs.msg import Crossing
from lama_msgs.msg import Frontier

g_max_length = 2

g_goal_reached = False
g_odom = Odometry()
g_odom.header.frame_id = 'odom'
g_transform_listener = None


def callback_goal_reached(msg):
    global g_goal_reached
    g_goal_reached = msg.data


def callback_odom(msg):
    global g_odom
    g_odom = msg


def main():
    global g_goal_reached
    crossing = Crossing()
    # We should have at least 3 frontiers for goto_crossing to go
    # to the crossing center.
    crossing.frontiers.append(Frontier())
    crossing.frontiers.append(Frontier())
    crossing.frontiers.append(Frontier())
    crossing_publisher = rospy.Publisher('~crossing', Crossing, queue_size=1)
    marker = Marker()
    marker.header.frame_id = 'odom'
    marker.type = marker.SPHERE
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.pose.orientation.w = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.a = 0.5
    marker_publisher = rospy.Publisher('~crossing_marker', Marker,
                                       queue_size=1, latch=True)
    while not rospy.is_shutdown():
        g_goal_reached = False
        x_center = g_max_length * 2 * (random.random() - 0.5)
        y_center = g_max_length * 2 * (random.random() - 0.5)
        target_pose = copy.deepcopy(g_odom.pose.pose)
        target_pose.position.x += x_center
        target_pose.position.y += y_center
        target_pose_stamped = PoseStamped(g_odom.header, target_pose)
        rospy.loginfo('New relative target: ({}, {})'.format(
            x_center, y_center))
        marker.pose.position.x = target_pose.position.x
        marker.pose.position.y = target_pose.position.y
        marker_publisher.publish(marker)
        r = rospy.Rate(10)
        while not g_goal_reached and not rospy.is_shutdown():
            target_pose_stamped.header = g_odom.header
            g_transform_listener.waitForTransform('base_link',
                                                  g_odom.header.frame_id,
                                                  g_odom.header.stamp,
                                                  rospy.Duration(0.1))
            rel_target_pose_stamped = g_transform_listener.transformPose(
                'base_link', target_pose_stamped)
            crossing.center.x = rel_target_pose_stamped.pose.position.x
            crossing.center.y = rel_target_pose_stamped.pose.position.y
            rospy.logdebug('Crossing: ({}, {})'.format(
                crossing.center.x, crossing.center.y))
            crossing_publisher.publish(crossing)
            r.sleep()
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('test_nj_oa', log_level=rospy.INFO)
    g_transform_listener = tf.TransformListener()
    g_transform_listener.waitForTransform('odom', 'base_link', rospy.Time(),
                                          rospy.Duration(5.0))

    rospy.Subscriber('~goal_reached', Bool, callback_goal_reached)
    rospy.Subscriber('~odom', Odometry, callback_odom)
    main()
    rospy.spin()
