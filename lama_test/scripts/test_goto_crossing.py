# -*- coding: utf-8 -*-
# Test the behavior of the goto_crossing with a random walk.

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
from goto_crossing.srv import ResetIntegrals
from goto_crossing.srv import ResetIntegralsRequest

_max_length = 2

_goal_reached = False
_odom = Odometry()
_odom.header.frame_id = 'odom'
_transform_listener = None
_reset_integrals_client = None


def callback_goal_reached(msg):
    global _goal_reached
    _goal_reached = msg.data


def callback_odom(msg):
    global _odom
    _odom = msg


def main():
    global _goal_reached
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
        _goal_reached = False
        srv = ResetIntegralsRequest()
        _reset_integrals_client.call(srv)
        x_center = _max_length * 2 * (random.random() - 0.5)
        y_center = _max_length * 2 * (random.random() - 0.5)
        target_pose = copy.deepcopy(_odom.pose.pose)
        target_pose.position.x += x_center
        target_pose.position.y += y_center
        target_pose_stamped = PoseStamped(_odom.header, target_pose)
        rospy.loginfo('New relative target: ({}, {})'.format(
            x_center, y_center))
        marker.pose.position.x = target_pose.position.x
        marker.pose.position.y = target_pose.position.y
        marker_publisher.publish(marker)
        r = rospy.Rate(10)
        while not _goal_reached and not rospy.is_shutdown():
            target_pose_stamped.header = _odom.header
            _transform_listener.waitForTransform('base_link',
                                                 _odom.header.frame_id,
                                                 _odom.header.stamp,
                                                 rospy.Duration(0.1))
            rel_target_pose_stamped = _transform_listener.transformPose(
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
    _transform_listener = tf.TransformListener()
    _transform_listener.waitForTransform('odom', 'base_link', rospy.Time(),
                                         rospy.Duration(5.0))

    _reset_integrals_client = rospy.ServiceProxy(
        'goto_crossing/reset_integrals', ResetIntegrals)
    rospy.Subscriber('~goal_reached', Bool, callback_goal_reached)
    rospy.Subscriber('~odom', Odometry, callback_odom)
    main()
    rospy.spin()
