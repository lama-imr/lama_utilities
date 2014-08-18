#!/usr/bin/env python

import rospy
from lama_interfaces.interface_factory import interface_factory

# example of call
if __name__ == '__main__':
    rospy.init_node('lama_interfaces', anonymous=True)
    interface_factory('double_descriptor', 'lama_interfaces/lmi_vector_double')
    interface_factory('polygon_descriptor', 'lama_interfaces/lmi_polygon')
    interface_factory('laser_descriptor',
                      'lama_interfaces/lmi_laser_descriptor')
    interface_factory('dummy_descriptor',
                      'lama_interfaces/lmi_dummy_descriptor')
    interface_factory('pose_descriptor',
                      'lama_interfaces/lmi_vector_pose')
    interface_factory('odometry_descriptor',
                      'lama_interfaces/lmi_vector_odometry')
    rospy.spin()
