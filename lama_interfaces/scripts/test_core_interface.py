#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import rospy

from lama_interfaces.core_interface import core_interface

rospy.init_node('lama_interfaces', anonymous=True)
core_interface()

rospy.spin()
