#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import unittest

import rospy
import roslib.message

from nlj_dummy.msg import Dummy

from lama_interfaces.interface_factory import interface_factory
from nlj_dummy.srv import GetDummyDescriptorRequest


class RosTestCase(unittest.TestCase):
    def assertMsgEqual(self, msg0, msg1):
        """Fail if two ROS messages are not equal"""
        self.assertIsInstance(msg0, roslib.message.Message,
                              msg='Argument 1 is not a Message')
        self.assertIsInstance(msg1, roslib.message.Message,
                              msg='Argument 2 is not a Message')
        slots0 = msg0.__slots__
        slots1 = msg1.__slots__
        self.assertEquals(slots0, slots1,
                          msg=('Messages do not have the same arguments\n' +
                               'Msg1 args: {}\n'.format(slots0) +
                               'Msg2 args: {}\n'.format(slots1)))
        for slot in slots0:
            value0 = getattr(msg0, slot)
            value1 = getattr(msg1, slot)
            if isinstance(value0, roslib.message.Message):
                self.assertMsgEqual(value0, value1)
            elif isinstance(value0, (list, tuple)):
                self.assertAlmostEquals(list(value0), list(value1), places=6,
                                        msg='Argument {} differ: {} != {}'.
                                        format(slot, value0, value1))
            else:
                self.assertAlmostEqual(value0, value1, places=6,
                                       msg='Argument {} differ: {} != {}'.
                                       format(slot, value0, value1))


class TestDbMessagePassing(RosTestCase):
    """test setting and getting dummy descriptors"""
    def __init__(self, *args, **kwargs):
        rospy.init_node('test_lama_interfaces_nlj_dummy', anonymous=True)
        super(TestDbMessagePassing, self).__init__(*args, **kwargs)

    def test_dummy(self):
        """Test passing and getting a Dummy message"""
        interface_name = 'dummy_descriptor'
        getter_service = 'nlj_dummy/GetDummyDescriptor'
        setter_service = 'nlj_dummy/SetDummyDescriptor'

        # Set up node as well as getter and setter services.
        iface = interface_factory(interface_name,
                                  getter_service,
                                  setter_service)
        get_srv = rospy.ServiceProxy(iface.getter_service_name,
                                     iface.getter_service_class)
        set_srv = rospy.ServiceProxy(iface.setter_service_name,
                                     iface.setter_service_class)

        dummy0 = Dummy()
        dummy0.value = 354
        dummy1 = Dummy()
        dummy1.value = 5649

        id_from_setter0 = set_srv(dummy0)
        # id_from_setter cannot be passed to get_srv because of
        # type incompatibility, "transform" it to a Get...Request()
        id_to_getter0 = GetDummyDescriptorRequest()
        id_to_getter0.id = id_from_setter0.id
        response0 = get_srv(id_to_getter0)

        id_from_setter1 = set_srv(dummy1)
        # id_from_setter cannot be passed to get_srv because of
        # type incompatibility, "transform" it to a Get...Request()
        id_to_getter1 = GetDummyDescriptorRequest()
        id_to_getter1.id = id_from_setter1.id
        response1 = get_srv(id_to_getter1)

        self.assertIsNot(dummy0, response0.descriptor)
        self.assertIsNot(dummy0.value, response0.descriptor.value)
        self.assertIsNot(dummy1, response1.descriptor)
        self.assertEqual(dummy0.value, response0.descriptor.value)
        self.assertEqual(dummy1.value, response1.descriptor.value)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('nlj_dummy',
                   'test_db_message_passing',
                   TestDbMessagePassing)
