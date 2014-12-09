#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import actionlib

from lama_jockeys.msg import NavigateAction
from lama_jockeys.msg import NavigateGoal

_edge_id = 1

if __name__ == '__main__':
    rospy.init_node('test_anj_featurenav')

    client = actionlib.SimpleActionClient('anj_featurenav_navigator',
                                          NavigateAction)
    client.wait_for_server()

    navigate_goal = NavigateGoal()
    navigate_goal.action = navigate_goal.TRAVERSE
    navigate_goal.edge.id = _edge_id
    rospy.loginfo('Starting navigation')
    client.send_goal_and_wait(navigate_goal)
    result = client.get_result()

    if result.final_state != result.DONE:
        rospy.logerr('Navigation error')
        exit()

    rospy.loginfo('Navigation successful')

    rospy.spin()
