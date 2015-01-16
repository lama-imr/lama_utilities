#!/usr/bin/env python

from __future__ import print_function

import random
import unittest

import rospy
import actionlib
from lama_jockeys.msg import NavigateAction
from lama_jockeys.msg import NavigateGoal
from lama_jockeys.msg import LocalizeAction
from lama_jockeys.msg import LocalizeGoal


def navigateFeedbackCallback(feedback):
    # TODO: Add a test on feedback.current_state and other
    # "enum" in devel/share/lama_jockeys/msg/NavigateFeedback.msg
    # states = ['', 'TRAVERSING', 'INTERRUPTED']
    # rospy.loginfo('Navigation current state: ' +
    #               states[feedback.current_state])
    # rospy.loginfo('Time elapsed: {:.3f} s'.format(
    #     feedback.time_elapsed.to_sec()))
    # rospy.loginfo('Completion: {:.2f} %'.format(feedback.completion * 100))
    pass


class TestNljdummy(unittest.TestCase):
    def test_nlj_dummy(self):
        navigate = actionlib.SimpleActionClient('navigating_jockey',
                                                NavigateAction)
        navigate.wait_for_server()
        localize = actionlib.SimpleActionClient('localizing_jockey',
                                                LocalizeAction)
        localize.wait_for_server()

        nav_goal = NavigateGoal()
        for i in range(2):
            loc_goal = LocalizeGoal()
            loc_goal.action = loc_goal.GET_VERTEX_DESCRIPTOR
            localize.send_goal_and_wait(loc_goal, rospy.Duration(0.1))
            loc_result = localize.get_result()
            self.assertIsNotNone(loc_result)

            loc_goal.action = loc_goal.GET_EDGES_DESCRIPTORS
            localize.send_goal_and_wait(loc_goal, rospy.Duration(0.5))
            loc_result = localize.get_result()
            self.assertIsNotNone(loc_result)
            self.assertIs(len(loc_result.descriptor_links), 4,
                          msg='expected 4 descriptor_links, got {}'.format(
                              len(loc_result.descriptor_links)))

            nav_goal.action = nav_goal.TRAVERSE
            random_descriptor = random.choice(loc_result.descriptor_links)
            nav_goal.descriptor_link = random_descriptor
            navigate.send_goal(nav_goal,
                               feedback_cb=navigateFeedbackCallback)
            navigate.wait_for_result(rospy.Duration.from_sec(5.0))
            nav_result = navigate.get_result()
            self.assertIsNotNone(nav_result)
            self.assertEqual(nav_result.final_state, nav_result.DONE,
                             msg='expected final_state DONE, got {}'.format(
                                 nav_result.final_state))

if __name__ == '__main__':
    rospy.init_node('test_nlj_dummy')

    import rostest
    rostest.rosrun('nlj_dummy',
                   'test_nlj_dummy',
                   TestNljdummy)
