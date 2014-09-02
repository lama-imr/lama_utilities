#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import pi
import unittest

import rospy
import roslib.message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

from lama_interfaces.interface_factory import interface_factory
from lama_interfaces.srv import lmi_vector_double_getRequest
from lama_interfaces.srv import GetVectorLaserScanRequest
from lama_interfaces.srv import lmi_vector_pose_getRequest
from lama_interfaces.srv import lmi_vector_odometry_getRequest
from lama_interfaces.srv import lmi_polygon_getRequest


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
    """test setting and getting several descriptors"""
    def test_vector_double(self):
        """Test for a message as a list or tuple"""
        interface_name = 'vector_double_descriptor'
        getter_service = 'lama_interfaces/lmi_vector_double_get'
        setter_service = 'lama_interfaces/lmi_vector_double_set'

        # Set up node as well as getter and setter services.
        rospy.init_node('lama_interfaces', anonymous=True)
        iface = interface_factory(interface_name, getter_service, setter_service)
        get_srv = rospy.ServiceProxy(iface.getter_service_name,
                                     iface.getter_service_class)
        set_srv = rospy.ServiceProxy(iface.setter_service_name,
                                     iface.setter_service_class)

        msg = (1.45, 5.9)

        descriptor_from_setter = set_srv(msg)
        # descriptor_from_setter cannot be passed to get_srv because of
        # type incompatibility, "transform" it to a ..._getRequest()
        descriptor_to_getter = lmi_vector_double_getRequest()
        descriptor_to_getter.id = descriptor_from_setter.id
        response = get_srv(descriptor_to_getter)

        self.assertIsNot(msg, response.descriptor)
        self.assertAlmostEquals(msg, response.descriptor, places=6)

        msg = [54.29, -9458.2]

        descriptor_from_setter = set_srv(msg)
        # descriptor_from_setter cannot be passed to get_srv because of
        # type incompatibility, "transform" it to a ..._getRequest()
        descriptor_to_getter = lmi_vector_double_getRequest()
        descriptor_to_getter.id = descriptor_from_setter.id
        response = get_srv(descriptor_to_getter)

        self.assertIsNot(msg, response.descriptor)
        self.assertAlmostEquals(msg, list(response.descriptor), places=6)

    def test_vector_laser_scan(self):
        """Test passing and getting a LaserScan[] message"""
        interface_name = 'laser_descriptor'
        getter_service = 'lama_interfaces/GetVectorLaserScan'
        setter_service = 'lama_interfaces/SetVectorLaserScan'

        # Set up node as well as getter and setter services.
        rospy.init_node('lama_interfaces', anonymous=True)
        iface = interface_factory(interface_name, getter_service, setter_service)
        get_srv = rospy.ServiceProxy(iface.getter_service_name,
                                     iface.getter_service_class)
        set_srv = rospy.ServiceProxy(iface.setter_service_name,
                                     iface.setter_service_class)

        scan0 = LaserScan()
        scan0.header.seq = 1
        scan0.header.stamp = rospy.Time.now()
        scan0.header.frame_id = 'frame0'
        scan0.angle_min = -pi
        scan0.angle_max = pi
        scan0.range_max = 10.
        scan0.ranges = [0., 1.]
        scan1 = LaserScan()
        scan1.header.seq = 2
        scan1.header.stamp = rospy.Time.now()
        scan1.header.frame_id = 'frame1'
        scan1.angle_min = -pi / 2
        scan1.angle_max = pi / 2
        scan1.range_max = 9.
        scan1.ranges = [2., 3.]
        out_scans = [scan0, scan1]

        descriptor_from_setter = set_srv(out_scans)
        # descriptor_from_setter cannot be passed to get_srv because of
        # type incompatibility, "transform" it to a ..._getRequest()
        descriptor_to_getter = GetVectorLaserScanRequest()
        descriptor_to_getter.id = descriptor_from_setter.id
        response = get_srv(descriptor_to_getter)

        self.assertEqual(len(out_scans), len(response.descriptor))
        self.assertIsNot(scan0, response.descriptor[0])
        self.assertIsNot(scan0, response.descriptor[1])
        self.assertIsNot(scan1, response.descriptor[0])
        self.assertIsNot(scan1, response.descriptor[1])
        for scan_out, scan_in in zip(out_scans, response.descriptor):
            self.assertMsgEqual(scan_out, scan_in)

    def test_pose(self):
        """Test passing and getting a Pose[] message"""
        interface_name = 'pose_descriptor'
        getter_service = 'lama_interfaces/lmi_vector_pose_get'
        setter_service = 'lama_interfaces/lmi_vector_pose_set'

        # Set up node as well as getter and setter services.
        rospy.init_node('lama_interfaces', anonymous=True)
        iface = interface_factory(interface_name, getter_service, setter_service)
        get_srv = rospy.ServiceProxy(iface.getter_service_name,
                                     iface.getter_service_class)
        set_srv = rospy.ServiceProxy(iface.setter_service_name,
                                     iface.setter_service_class)

        pose0 = Pose()
        pose0.position.x = 34.5
        pose0.position.y = 5424.9
        pose0.position.z = -1238.5
        pose0.orientation.x = 0.5215973586107343
        pose0.orientation.y = 0.5145501061145893
        pose0.orientation.z = 0.20795107752790498
        pose0.orientation.w = (1 - (
            pose0.orientation.x ** 2 +
            pose0.orientation.y ** 2 +
            pose0.orientation.z ** 2)) ** 0.5
        poses = [pose0, pose0]

        descriptor_from_setter = set_srv(poses)
        # descriptor_from_setter cannot be passed to get_srv because of
        # type incompatibility, "transform" it to a ..._getRequest()
        descriptor_to_getter = lmi_vector_pose_getRequest()
        descriptor_to_getter.id = descriptor_from_setter.id
        response = get_srv(descriptor_to_getter)

        self.assertEqual(len(poses), len(response.descriptor))
        self.assertIsNot(poses[0], response.descriptor[0])
        self.assertIsNot(poses[1], response.descriptor[1])
        for pose_out, pose_in in zip(poses, response.descriptor):
            self.assertMsgEqual(pose_out, pose_in)

    def test_odometry(self):
        """Test passing and getting a Odometry[] message"""
        interface_name = 'odometry_descriptor'
        getter_service = 'lama_interfaces/lmi_vector_odometry_get'
        setter_service = 'lama_interfaces/lmi_vector_odometry_set'

        # Set up node as well as getter and setter services.
        rospy.init_node('lama_interfaces', anonymous=True)
        iface = interface_factory(interface_name, getter_service, setter_service)
        get_srv = rospy.ServiceProxy(iface.getter_service_name,
                                     iface.getter_service_class)
        set_srv = rospy.ServiceProxy(iface.setter_service_name,
                                     iface.setter_service_class)

        odom = Odometry()
        odom.header.seq = 1
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'frame'
        odom.pose.pose.position.x = 34.5
        odom.pose.pose.position.y = 5424.9
        odom.pose.pose.position.z = -1238.5
        odom.pose.pose.orientation.x = 0.5215973586107343
        odom.pose.pose.orientation.y = 0.5145501061145893
        odom.pose.pose.orientation.z = 0.20795107752790498
        odom.pose.pose.orientation.w = (1 - (
            odom.pose.pose.orientation.x ** 2 +
            odom.pose.pose.orientation.y ** 2 +
            odom.pose.pose.orientation.z ** 2)) ** 0.5
        odom.twist.twist.linear.x = 43587.2
        odom.twist.twist.linear.y = 23.0
        odom.twist.twist.linear.z = -34.1
        odom.twist.twist.angular.x = 0.9
        odom.twist.twist.angular.y = 1.459
        odom.twist.twist.angular.z = 6.8
        odom.pose.covariance = [0.66661561, 0.66656431, 0.98260750,
                                0.32037977, 0.37148378, 0.39368085,
                                0.79881689, 0.62314512, 0.69425357,
                                0.93326911, 0.68453431, 0.17272260,
                                0.78877475, 0.47810612, 0.81750745,
                                0.07891373, 0.43773912, 0.87397257,
                                0.07260977, 0.28417538, 0.64133374,
                                0.14946342, 0.21064081, 0.64976447,
                                0.04895663, 0.59137640, 0.91207933,
                                0.13848898, 0.68960110, 0.05312844,
                                0.67371487, 0.73003252, 0.81269526,
                                0.73006462, 0.31009889, 0.21264097]

        odom.twist.covariance = [0.74771120, 0.67419758, 0.94429162,
                                 0.50301016, 0.44133241, 0.67792401,
                                 0.69450461, 0.01300356, 0.96009616,
                                 0.20118201, 0.36520037, 0.53622379,
                                 0.40432863, 0.96075259, 0.63216066,
                                 0.73726525, 0.77320984, 0.38823612,
                                 0.24819383, 0.41003267, 0.14771474,
                                 0.27845271, 0.44038160, 0.11593736,
                                 0.19048534, 0.15305760, 0.29514903,
                                 0.87055584, 0.24909447, 0.57612981,
                                 0.50363839, 0.65519450, 0.05121275,
                                 0.01989313, 0.06818292, 0.46997481]
        odoms = [odom]

        descriptor_from_setter = set_srv(odoms)
        # descriptor_from_setter cannot be passed to get_srv because of
        # type incompatibility, "transform" it to a ..._getRequest()
        descriptor_to_getter = lmi_vector_odometry_getRequest()
        descriptor_to_getter.id = descriptor_from_setter.id
        response = get_srv(descriptor_to_getter)

        self.assertEqual(len(odoms), len(response.descriptor))
        self.assertIsNot(odoms[0], response.descriptor[0])
        for odom_out, odom_in in zip(odoms, response.descriptor):
            self.assertMsgEqual(odom_out, odom_in)

    def test_polygon(self):
        """Test passing and getting a Polygon message"""
        interface_name = 'poly'
        getter_service = 'lama_interfaces/lmi_polygon_get'
        setter_service = 'lama_interfaces/lmi_polygon_set'

        # Set up node as well as getter and setter services.
        rospy.init_node('lama_interfaces', anonymous=True)
        iface = interface_factory(interface_name, getter_service, setter_service)
        get_srv = rospy.ServiceProxy(iface.getter_service_name,
                                     iface.getter_service_class)
        set_srv = rospy.ServiceProxy(iface.setter_service_name,
                                     iface.setter_service_class)

        polygon = Polygon()

        descriptor_from_setter = set_srv(polygon)
        # descriptor_from_setter cannot be passed to get_srv because of
        # type incompatibility, "transform" it to a ..._getRequest()
        descriptor_to_getter = lmi_polygon_getRequest()
        descriptor_to_getter.id = descriptor_from_setter.id
        response = get_srv(descriptor_to_getter)

        self.assertIsNot(polygon, response.descriptor)
        for point_out, point_in in zip(polygon.points, response.descriptor.points):
            self.assertAlmostEqual(point_out.x, point_in.x, places=5)
            self.assertAlmostEqual(point_out.y, point_in.y, places=5)
            self.assertAlmostEqual(point_out.z, point_in.z, places=5)

        polygon = Polygon()
        polygon.points.append(Point32())
        polygon.points[0].x = 45.6
        polygon.points[0].y = -34.0
        polygon.points[0].z = -26.4
        polygon.points.append(Point32())
        polygon.points[1].x = -5.6
        polygon.points[1].y = -3.0
        polygon.points[1].z = 6.4

        descriptor_from_setter = set_srv(polygon)
        # descriptor_from_setter cannot be passed to get_srv because of
        # type incompatibility, "transform" it to a ..._getRequest()
        descriptor_to_getter = lmi_polygon_getRequest()
        descriptor_to_getter.id = descriptor_from_setter.id
        response = get_srv(descriptor_to_getter)

        self.assertIsNot(polygon, response.descriptor)
        for point_out, point_in in zip(polygon.points, response.descriptor.points):
            self.assertAlmostEqual(point_out.x, point_in.x, places=5)
            self.assertAlmostEqual(point_out.y, point_in.y, places=5)
            self.assertAlmostEqual(point_out.z, point_in.z, places=5)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('lama_interfaces',
                   'test_db_message_passing',
                   TestDbMessagePassing)
