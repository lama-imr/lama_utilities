#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

from StringIO import StringIO

from lama_msgs.msg import Crossing
from lama_msgs.msg import Frontier

from crossing_detector_wrapper_cpp import CrossingDetectorWrapper


class CrossingDetector(object):
    def __init__(self, frontier_width, max_frontier_angle=0.785):
        self._detector = CrossingDetectorWrapper(frontier_width,
                                                 max_frontier_angle)

    def _to_cpp(self, msg):
        """Return a serialized string from a ROS message

        Parameters
        ----------
        - msg: a ROS message instance.
        """
        buf = StringIO()
        msg.serialize(buf)
        return buf.getvalue()

    def _from_cpp(self, str_msg, cls):
        """Return a ROS message from a serialized string

        Parameters
        ----------
        - str_msg: str, serialized message
        - cls: ROS message class, e.g. sensor_msgs.msg.LaserScan.
        """
        msg = cls()
        return msg.deserialize(str_msg)

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
        str_profile = self._to_cpp(profile)
        str_frontiers = self._detector.frontiers(str_profile, normalize)
        frontiers = []
        for str_frontier in str_frontiers:
            frontiers.append(self._from_cpp(str_frontier, Frontier))
        return frontiers
