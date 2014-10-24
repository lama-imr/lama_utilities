#!/usr/bin/python

import rospy

from lama_interfaces import core_interface
from lama_interfaces.msg import LamaObject
from lama_interfaces.srv import ActOnMap
from lama_interfaces.srv import ActOnMapRequest

g_engine_name = rospy.get_param('/database_engine', 'sqlite:///created.sql')
g_iface = core_interface.CoreDBInterface(g_engine_name)
g_map_agent = rospy.ServiceProxy(g_iface.action_service_name, ActOnMap)


def _get_graph_key(graph, id_):
    """Return the key, a LamaObject, that has id_ as id
    """
    for lama_object in graph.iterkeys():
        if lama_object.id == id_:
            return lama_object
    return None


def get_directed_graph():
    """Return the directed graph as a dict {vertex: [edge0, edge1, ...], ...}

    Return the directed graph as a dict {vertex: [edge0, edge1, ...], ...},
    where vertex is a LamaObject of type vertex and edge0 is a LamaObject
    of type edge and with edge0.references[0] = vertex.
    All vertices are listed as key. All edges are listed as values.
    """
    # Get the vertices (graph keys).
    map_action = ActOnMapRequest()
    map_action.action = map_action.GET_VERTEX_LIST
    response = g_map_agent.call(map_action)
    graph = {}
    for vertices in response.objects:
        graph[vertices] = []
    # Get the edges (graph values).
    map_action = ActOnMapRequest()
    map_action.action = map_action.GET_EDGE_LIST
    response = g_map_agent.call(map_action)
    graph = {}
    for edge in response.objects:
        first_vertex = _get_graph_key(graph, edge.references[0])
        if first_vertex is None:
            rospy.logerr(('Vertex {} does not exist although ' +
                          'it is the first vertex of edge {}').format(
                              edge.references[0],
                              edge.id))
            continue
        graph[first_vertex].append(edge)


def get_edge_with_vertices(v0, v1):
    """Return the edge from v0 to v1, as LamaObject"""
    map_action = ActOnMapRequest()
    map_action.action = map_action.GET_EDGE_LIST
    response = g_map_agent(map_action)
    for edge in response.objects:
        if (edge.references[0] == v0) and (edge.references[1] == v1):
            return edge
    return None


def get_descriptors(object_id, interface, getter):
    """Retrieve the descriptors associated with LamaObject with id object_id

    Return a list of descriptors associated with getter.

    Parameters
    ----------
    - object_id: int, LamaObject's id
    - interface: str, interface name associated with getter
    - getter: ROS ServiceProxy for a ROS message type
    """
    map_action = ActOnMapRequest()
    map_action.action = map_action.GET_DESCRIPTOR_LINKS
    lama_object = LamaObject()
    lama_object.id = object_id
    map_action.object = lama_object
    map_action.interface_name = interface
    response = g_map_agent(map_action)
    descriptors = []
    for descriptor_link in response.descriptor_links:
        descriptor = getter(descriptor_link.descriptor_id)
        descriptors.append(descriptor)
    return descriptors
