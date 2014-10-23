# -*- coding: utf-8 -*-
# Wrapper class for CrossingDetector, meant to be used directly.
#
# Before creating an instance of CrossingDetector, roscpp must be
# initialized. This can be achieved for example with
# from moveit_ros_planning_interface._moveit_roscpp_initializer \
#   import roscpp_init
# roscpp_init('node_name', [])

import rospy

from lama_msgs.msg import Crossing
from lama_msgs.msg import Frontier
from lama_msgs.msg import PlaceProfile

from crossing_detector_wrapper_cpp import CrossingDetectorWrapper
from wrapper import WrapperPy


class CrossingDetector(WrapperPy):
    def __init__(self, frontier_width, max_frontier_angle=0.785):
        self._detector = CrossingDetectorWrapper(frontier_width,
                                                 max_frontier_angle)

    def crossingDescriptor(self, profile, normalize=False):
        """Return a Crossing message from analysis of a PlaceProfile

        Return a lama_msgs/Crossing message from analysis of a
        lama_msgs/PlaceProfile.

        Parameters
        ----------
        - profile: a lama_msgs/PlaceProfile message
        - normalize: true if the PlaceProfile should be normalized in a first
            step.
        """
        if not isinstance(profile, PlaceProfile):
            err = 'Argument 1 is not a lama_msgs/PlaceProfile instance'
            rospy.logerr(err)
            rospy.ROSException(err)
        str_profile = self._to_cpp(profile)
        str_crossing = self._detector.crossingDescriptor(str_profile, normalize)
        return self._from_cpp(str_crossing, Crossing)

    def frontiers(self, profile, normalize=False):
        """Return a list of Frontier messages from analysis of a PlaceProfile

        Return a list of lama_msgs/Frontier messages from analysis of a
        lama_msgs/PlaceProfile.

        Parameters
        ----------
        - profile: a lama_msgs/PlaceProfile message
        - normalize: true if the PlaceProfile should be normalized in a first
            step.
        """
        if not isinstance(profile, PlaceProfile):
            err = 'Argument 1 is not a lama_msgs/PlaceProfile instance'
            rospy.logerr(err)
            rospy.ROSException(err)
        str_profile = self._to_cpp(profile)
        str_frontiers = self._detector.frontiers(str_profile, normalize)
        frontiers = []
        for str_frontier in str_frontiers:
            frontiers.append(self._from_cpp(str_frontier, Frontier))
        return frontiers
