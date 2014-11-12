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

## A sample python unit test
class TestBareBones(unittest.TestCase):    

    def test001_one_equals_one(self):
       """test message"""
       self.assertEquals(1, 1, "1!=1")
    def test002_push_vertex(self):
       """Test ActOnMap service push vertex:"""

       rospy.wait_for_service('lama_map_agent')
       map_agent = rospy.ServiceProxy('lama_map_agent', ActOnMap )
       try:
          response = map_agent(object=vertex1, action=ActOnMapRequest.PUSH_VERTEX)
	  global v1_id 
	  v1_id = response.objects[0].id
       except rospy.ServiceException as exc:
	  print("Service did not process request: " + str(exc))
	  self.assertTrue(False,"service did not process request: " + str(exc))

    def test003_pull_vertex(self):
       """Test ActOnMap service pull vertex: """
       rospy.wait_for_service('lama_map_agent')
       map_agent = rospy.ServiceProxy('lama_map_agent', ActOnMap )
       req = LamaObject()
       req.id = v1_id
       response=map_agent(object=req, action=ActOnMapRequest.PULL_VERTEX)
       self.assertEquals(response.objects[0].id_in_world, vertex1.id_in_world, "id_in_world is not equal")
       self.assertEquals(response.objects[0].name, vertex1.name, "id_in_world is not equal")



if __name__ == '__main__':

   import rosunit
   rosunit.unitrun(PKG, 'test_bare_bones', TestBareBones)
