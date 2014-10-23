# -*- coding: utf-8 -*-
# Wrapper class for LaserCrossingDetector, meant to be used directly.
#
# Before creating an instance of LaserCrossingDetector, roscpp must be
# initialized. This can be achieved for example with
# from moveit_ros_planning_interface._moveit_roscpp_initializer \
#   import roscpp_init
# roscpp_init('node_name', [])

from StringIO import StringIO

import rospy
from sensor_msgs.msg import LaserScan
from lama_msgs.msg import Crossing
from lama_msgs.msg import Frontier

from laser_crossing_detector_wrapper_cpp import LaserCrossingDetectorWrapper
from wrapper import WrapperPy


class LaserCrossingDetector(WrapperPy):
    def __init__(self, frontier_width, max_frontier_angle=0.785):
        self._detector = LaserCrossingDetectorWrapper(frontier_width,
                                                      max_frontier_angle)

    def crossingDescriptor(self, scan, normalize=False):
        """Return a Crossing message from analysis of a LaserScan

        Return a lama_msgs/Crossing message from analysis of a
        sensor_msgs/LaserScan.

        Parameters
        ----------
        - scan: a sensor_msgs/LaserScan message
        - normalize: true if scan should be normalized in a first step.
        """
        if not isinstance(scan, LaserScan):
            err = 'Argument 1 is not a sensor_msgs/LaserScan instance'
            rospy.logerr(err)
            rospy.ROSException(err)
        str_scan = self._to_cpp(scan)
        str_crossing = self._detector.crossingDescriptor(str_scan, normalize)
        return self._from_cpp(str_crossing, Crossing)

    def frontiers(self, scan, normalize=False):
        """Return a list of Frontier messages from analysis of a LaserScan

        Return a list of lama_msgs/Frontier messages from analysis of a
        sensor_msgs/LaserScan.

        Parameters
        ----------
        - scan: a sensor_msgs/LaserScan message
        - normalize: true if scan should be normalized in a first step.
        """
        if not isinstance(scan, LaserScan):
            err = 'Argument 1 is not a sensor_msgs/LaserScan instance'
            rospy.logerr(err)
            rospy.ROSException(err)
        str_scan = self._to_cpp(scan)
        str_frontiers = self._detector.frontiers(str_scan, normalize)
        frontiers = []
        for str_frontier in str_frontiers:
            frontiers.append(self._from_cpp(str_frontier, Frontier))
        return frontiers
