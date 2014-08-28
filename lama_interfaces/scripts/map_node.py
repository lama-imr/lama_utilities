#!/usr/bin/env python

import rospy

from lama_interfaces.core_interface import core_interface
from lama_interfaces.interface_factory import interface_factory
from lama_interfaces.srv import AddInterface
from lama_interfaces.srv import AddInterfaceResponse


def handle_add_interface(req):
    response = AddInterfaceResponse()
    try:
        iface = interface_factory(req.service_name, req.message_type)
    except ValueError:
        response.success = False
    response.success = True
    response.get_service_name = iface.srv_name + '_getter'
    response.set_service_name = iface.srv_name + '_setter'
    return response


if __name__ == '__main__':
    rospy.init_node('lama_interfaces', anonymous=True)

    # Add the core interface for lama objects and descriptor_id objects.
    core_interface()

    # Add the server for AddInterface.
    s = rospy.Service('interface_factory', AddInterface, handle_add_interface)

    # interface_factory('double_descriptor', 'lama_interfaces/lmi_vector_double')
    # interface_factory('polygon_descriptor', 'lama_interfaces/lmi_polygon')
    # interface_factory('laser_descriptor',
    #                   'lama_interfaces/lmi_laser_descriptor')
    # interface_factory('dummy_descriptor',
    #                   'lama_interfaces/lmi_dummy_descriptor')
    # interface_factory('pose_descriptor',
    #                   'lama_interfaces/lmi_vector_pose')
    # interface_factory('odometry_descriptor',
    #                   'lama_interfaces/lmi_vector_odometry')
    rospy.spin()
