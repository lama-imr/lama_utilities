#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import actionlib

from lama_jockeys.msg import NavigateAction
from lama_jockeys.msg import NavigateGoal

if __name__ == '__main__':
    rospy.init_node('test_nj_oa')

    client = actionlib.SimpleActionClient('jockey_server', NavigateAction)
    client.wait_for_server(rospy.Duration(5))

    nav_goal = NavigateGoal()
    nav_goal.action = nav_goal.TRAVERSE
    client.send_goal(nav_goal)

    # Wait indefinitely.
    client.wait_for_result()

    rospy.spin()
