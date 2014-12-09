#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import actionlib

from lama_jockeys.msg import LearnAction
from lama_jockeys.msg import LearnGoal

if __name__ == '__main__':
    rospy.init_node('test_anj_featurenav')

    client = actionlib.SimpleActionClient('anj_featurenav_learner',
                                          LearnAction)
    client.wait_for_server()

    learn_goal = LearnGoal()
    learn_goal.action = learn_goal.START_LEARN
    client.send_goal(learn_goal)

    rospy.loginfo('Learning started')

    # Wait 5.
    rospy.sleep(5)

    learn_goal = LearnGoal()
    learn_goal.action = learn_goal.STOP_LEARN
    client.send_goal(learn_goal)

    rospy.loginfo('Learning finished')

    rospy.spin()
