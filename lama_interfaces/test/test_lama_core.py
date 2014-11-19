#!/usr/bin/env python
PKG='lama_core_interface'

import rospy

import sys
import unittest
import lama_interfaces
from lama_interfaces.msg import *
from lama_interfaces.srv import *

v1_id = -1
vertex1 = LamaObject()
vertex1.id_in_world = 1
vertex1.name = "vertex1"
vertex1.type=LamaObject.VERTEX

vertex2 = LamaObject()
vertex2.id_in_world = 2
vertex2.name = "vertex2"
vertex2.type=LamaObject.VERTEX

vertex3 = LamaObject()
vertex3.id_in_world = 3
vertex3.name = "vertex3"
vertex3.type=LamaObject.VERTEX

vertex3a = LamaObject()
vertex3a.id_in_world = 4 
vertex3a.name = "vertex3"
vertex3a.type=LamaObject.VERTEX

vertex3b = LamaObject()
vertex3b.id_in_world = 3 
vertex3b.name = "vertex3b"
vertex3b.type=LamaObject.VERTEX




## A sample python unit test
class TestBareBones(unittest.TestCase):    

    def test001_one_equals_one(self):
        """test message"""
        self.assertEquals(1, 1, "1!=1")\

    def test002_push_vertex(self):
        """Test ActOnMap service push vertex:"""
        rospy.wait_for_service('lama_map_agent')
        map_agent = rospy.ServiceProxy('lama_map_agent', ActOnMap )
        try:
            response = map_agent(object=vertex1, action=ActOnMapRequest.PUSH_VERTEX)
            global v1_id 
            v1_id = response.objects[0].id
        except rospy.ServiceException as exc:
            self.assertTrue(False,"service did not process request: " + str(exc))
        
        try:
            response = map_agent(object=vertex2, action=ActOnMapRequest.PUSH_VERTEX)
        except rospy.ServiceException as exc:
            self.assertTrue(False,"service did not process request vertex2: " + str(exc))

        try:
            response = map_agent(object=vertex3, action=ActOnMapRequest.PUSH_VERTEX)
        except rospy.ServiceException as exc:
            self.assertTrue(False,"service did not process request vertex3: " + str(exc))

        try:
            response = map_agent(object=vertex3a, action=ActOnMapRequest.PUSH_VERTEX)
        except rospy.ServiceException as exc:
            self.assertTrue(False,"service did not process request vertex3a: " + str(exc))

        try:
            response = map_agent(object=vertex3b, action=ActOnMapRequest.PUSH_VERTEX)
        except rospy.ServiceException as exc:
            self.assertTrue(False,"service did not process request vertex3b: " + str(exc))



    def test003_pull_vertex(self):
        """Test ActOnMap service pull vertex: """
        rospy.wait_for_service('lama_map_agent')
        map_agent = rospy.ServiceProxy('lama_map_agent', ActOnMap )
        req = LamaObject()
        req.id = v1_id
        response=map_agent(object=req, action=ActOnMapRequest.PULL_VERTEX)
        self.assertEquals(response.objects[0].id_in_world, vertex1.id_in_world, "id_in_world is not equal")
        self.assertEquals(response.objects[0].name, vertex1.name, "name is not equal") 
    def test004_pull_vertex_id_in_world(self):
        """Test ActOnMap service pull vertex according world id: """
        rospy.wait_for_service('lama_map_agent')
        map_agent = rospy.ServiceProxy('lama_map_agent', ActOnMap )
        req = LamaObject()
        req.id_in_world = 1
        response=map_agent(object=req, action=ActOnMapRequest.PULL_VERTEX)
        self.assertEquals(response.objects[0].id_in_world, vertex1.id_in_world, "id_in_world is not equal")
        self.assertEquals(response.objects[0].name, vertex1.name, "name is not equal") 
    def test005_pull_vertex_name(self):
        """Test ActOnMap service pull vertex according world id: """
        rospy.wait_for_service('lama_map_agent')
        map_agent = rospy.ServiceProxy('lama_map_agent', ActOnMap )
        req = LamaObject()
        req.name = "vertex1"
        response=map_agent(object=req, action=ActOnMapRequest.PULL_VERTEX)
        self.assertEquals(response.objects[0].id_in_world, vertex1.id_in_world, "id_in_world is not equal")
        self.assertEquals(response.objects[0].name, vertex1.name, "name is not equal") 





if __name__ == '__main__':

   import rosunit
   rosunit.unitrun(PKG, 'test_bare_bones', TestBareBones)
