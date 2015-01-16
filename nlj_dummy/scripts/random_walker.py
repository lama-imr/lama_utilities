#!/usr/bin/env python

from __future__ import print_function

import random

import rospy
import actionlib
from lama_jockeys.msg import NavigateAction
from lama_jockeys.msg import NavigateGoal
from lama_jockeys.msg import LocalizeAction
from lama_jockeys.msg import LocalizeGoal


def navigateFeedbackCallback(feedback):
    # "enum" in devel/share/lama_jockeys/msg/NavigateFeedback.msg
    states = ['', 'TRAVERSING', 'INTERRUPTED']
    rospy.loginfo('Navigation current state: ' + states[feedback.current_state])
    rospy.loginfo('Time elapsed: {:.3f} s'.format(feedback.time_elapsed.to_sec()))
    rospy.loginfo('Completion: {:.2f} %'.format(feedback.completion * 100))

if __name__ == '__main__':
    rospy.init_node('random_walker')

    navigate = actionlib.SimpleActionClient('navigating_jockey', NavigateAction)
    navigate.wait_for_server()
    localize = actionlib.SimpleActionClient('localizing_jockey', LocalizeAction)
    localize.wait_for_server()

    nav_goal = NavigateGoal()
    try:
        while True:
            loc_goal = LocalizeGoal()
            loc_goal.action = loc_goal.GET_VERTEX_DESCRIPTOR
            localize.send_goal_and_wait(loc_goal, rospy.Duration(0.1))
            loc_result = localize.get_result()
            if not loc_result:
                rospy.logerr('Did not receive vertex descriptor within 0.1 s exiting')
                break
            rospy.loginfo('Received vertex descriptor')

            loc_goal.action = loc_goal.GET_EDGES_DESCRIPTORS
            localize.send_goal_and_wait(loc_goal, rospy.Duration(0.5))
            loc_result = localize.get_result()
            if not loc_result:
                rospy.logerr('Did not receive edge descriptors within 0.5 s exiting')
                break
            rospy.loginfo('Received {} descriptors in {:.3f} s'.format(
                len(loc_result.descriptor_links),
                loc_result.completion_time.to_sec()))

            nav_goal.action = nav_goal.TRAVERSE
            random_descriptor = random.choice(loc_result.descriptor_links)
            rospy.loginfo('Setting new goal: {}'.format(
                random_descriptor.descriptor_id))
            nav_goal.descriptor_link = random_descriptor
            navigate.send_goal(nav_goal,
                               feedback_cb=navigateFeedbackCallback)
            navigate.wait_for_result(rospy.Duration.from_sec(5.0))
            nav_result = navigate.get_result()
            if nav_result and nav_result.final_state == nav_result.DONE:
                print('Traversed egde {} in {:.2f} s'.format(
                    nav_goal.descriptor_link.descriptor_id,
                    nav_result.completion_time.to_sec()))
            else:
                if rospy.is_shutdown():
                    break
                print('Waited 5 sec for action server without success, retrying')
    except rospy.ServiceException, e:
        print('Service call failed: {}'.format(e))
