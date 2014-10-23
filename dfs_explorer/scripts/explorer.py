#!/usr/bin/python
# -*- coding: utf-8 -*-

# Implement a Depth-First Search explorer.
# Works in combination with lj_laser_heading, nj_laser, and nj_escape_crossing
# jockeys.

from __future__ import print_function, division

from math import pi

import rospy
import actionlib
from std_msgs.msg import Float32

from lama_jockeys.msg import NavigateAction
from lama_jockeys.msg import NavigateGoal
from lama_jockeys.msg import LocalizeAction
from lama_jockeys.msg import LocalizeGoal
from lama_msgs.srv import GetCrossing
from lama_interfaces.srv import ActOnMap
from lama_interfaces.srv import ActOnMapRequest
from lama_interfaces.core_interface import CoreDBInterface
from lama_interfaces.interface_factory import interface_factory
from lama_interfaces.graph_builder import get_edge_with_vertices

from graph import GraphTransformer

# TODO: add a mechanism for the robot not to come back to the vertex it comes
# from.

g_max_dissimilarity_for_same = 0.1


def normalize_angles(angles):
    def normalize_angle(angle):
        return (angle + pi) % (2 * pi) - pi
    return [normalize_angle(a) for a in angles]


def debug(msg):
    rospy.logdebug('{}: {}'.format(rospy.get_name(), msg))


def jockey_client(jockey_name, action_type):
    client = actionlib.SimpleActionClient(jockey_name, action_type)
    while not client.wait_for_server(rospy.Duration(5)):
        rospy.loginfo('Waiting for the jockey action ' +
                      'server ({})'.format(jockey_name))
    debug('communicating with the jockey ' +
          'action server {}'.format(jockey_name))
    return client


class ExplorerNode(object):
    def __init__(self):
        # Node and server initialization.
        rospy.init_node('dfs_explorer', log_level=rospy.DEBUG)
        navigating_jockey_name = rospy.get_param('~navigating_jockey_name',
                                                 'navigating_jockey')
        localizing_jockey_name = rospy.get_param('~localizing_jockey_name',
                                                 'localizing_jockey')
        escape_jockey_name = rospy.get_param('~escape_jockey_name',
                                             'nj_escape_jockey')

        # Navigate jockey server.
        self.navigate = jockey_client(navigating_jockey_name, NavigateAction)

        # Localize jockey server.
        self.localize = jockey_client(localizing_jockey_name, LocalizeAction)

        # Crossing escape jockey server.
        self.escape = jockey_client(escape_jockey_name, NavigateAction)

        # Map agent server.
        engine_name = rospy.get_param('/database_engine',
                                      'sqlite:///created.sql')
        iface = CoreDBInterface(engine_name)
        self.map_agent = rospy.ServiceProxy(iface.action_service_name, ActOnMap)

        # Descriptor getter for Crossing.
        self.crossing_interface_name = rospy.get_param(
            '~crossing_interface', 'crossing')
        crossing_getter_name = self.crossing_interface_name + '_getter'
        self.crossing_getter = rospy.ServiceProxy(crossing_getter_name,
                                                  GetCrossing)
        self.crossing_getter.wait_for_service()

        # Exit angles getter and setter (double).
        exit_angles_interface_name = rospy.get_param(
            'exit_angles_interface_name',
            'dfs_explorer_exit_angles')
        exits_iface = interface_factory(
            exit_angles_interface_name,
            'lama_interfaces/GetDouble',
            'lama_interfaces/SetDouble')
        self.exit_angles_getter = exits_iface.getter_service_proxy
        self.exit_angles_setter = exits_iface.setter_service_proxy

        # Exit angle topic advertiser.
        exit_angle_topic_name = rospy.get_param("exit_angle_topic",
                                                "exit_angle")
        self.exit_angle_publisher = rospy.Publisher(exit_angle_topic_name,
                                                    Float32,
                                                    queue_size=1,
                                                    latch=True)

        self.first_crossing_reached = False
        self.first_vertex = None
        self.last_vertex = None
        self.exit_taken = None
        self.next_vertex = None
        self.next_exit = None
        # The graph is organized as a map
        # vertex: [[vertex, exit_angle], [vertex, exit_angle], ...].
        # Where the second vertex is the vertex that will be at the next
        # crossing center when traversing edge (corridor) at absolute angle
        # exit_angle.
        # When starting to traverse an edge, vertex is set to None. A vertex
        # will be visited when all its neighbor vertices are not None.
        # The graph is then an oriented graph where the information for edge
        # a to b is the exit angle that was taken from a to reach b.
        self.graph_transformer = GraphTransformer(self.map_agent,
                                                  self.crossing_getter,
                                                  self.crossing_interface_name,
                                                  self.exit_angles_getter,
                                                  exit_angles_interface_name)

        debug('initialized')

    def move_to_crossing(self):
        """Move the robot to the first crossing
        Move the robot to the first crossing so that we can have a descriptor
        list to start with with the DFS algorithm.
        """
        debug('moving to crossing')
        nav_goal = NavigateGoal()
        nav_goal.action = nav_goal.TRAVERSE
        self.navigate.send_goal(nav_goal)
        self.navigate.wait_for_result()
        nav_result = self.navigate.get_result()
        if nav_result.final_state == nav_result.DONE:
            debug(('traversed to crossing center in {:.2f} s').format(
                nav_result.completion_time.to_sec()))
        else:
            err = '{}: something wrong happened, exiting!'.format(
                rospy.get_name())
            rospy.logerr(err)
            raise Exception(err)
        self.first_crossing_reached = True

    def loop(self):
        """Run the DFS algorithm until all vertices are visited

        Procedure:
        1. Get a new vertex descriptor when finished traversing, in the case
           that the vertex was not already visited.
        2. Choose the vertex with the next exit to visit and the direction to
           move with DFS.
        3. Move to that vertex.
        4. Let the robot escape from the node in the chosen direction.
        5. Let the navigating jockey move to the next crossing.
        6. Repeat from 1. indefinitely.
        """
        if not self.first_crossing_reached:
            rospy.logerr('Go to first crossing first')
            raise Exception('Go to first crossing first')

        while True:
            # 1. Get a new vertex descriptor when finished traversing.
            debug('getting descriptor')
            if not self.get_current_descriptor():
                rospy.logwarn('No descriptor, exiting')
                break

            # 2. Choose the vertex with the next exit to visit
            vertex_and_angle = self.get_next_vertex_to_visit()
            if vertex_and_angle is None:
                rospy.loginfo('I visisted all crossings, successfully exiting')
                break
            self.next_vertex, self.next_exit = vertex_and_angle
            debug('next vertex to visit {} (exit angle: {})'.format(
                self.next_vertex, self.next_exit))

            # 3. Move to that vertex.
            self.move_to_next_crossing()

            # 4. Let the robot escape from the node in the chosen direction.
            # The edge does not exists yet, set the direction through a topic.
            self.escape_from_crossing()

            # 5. Let the navigating jockey move to the next crossing.
            self.move_to_crossing()

    def get_current_descriptor(self):
        """Get the descriptors from the current crossing center

        Get the descriptors.
        Push the vertex.
        Add vertices to the graph.

        Parameters
        ----------
        - origin_angle: float, absolute angle of the exit the robot took
            when starting from origin_vertex.
        """
        loc_goal = LocalizeGoal()
        loc_goal.action = loc_goal.GET_VERTEX_DESCRIPTOR
        self.localize.send_goal_and_wait(loc_goal, rospy.Duration(0.5))
        loc_result = self.localize.get_result()
        if not loc_result:
            rospy.logerr('{}: did not receive vertex descriptor within ' +
                         '0.5 s, exiting'.format(rospy.get_name()))
            return False
        debug('received {} vertex descriptors'.format(
            len(loc_result.descriptor_links)))
        self.add_vertex(loc_result.descriptor_links)
        return True

    def add_vertex(self, descriptor_links):
        vertices, dissimilarities = self.get_dissimilarity()
        vertex_is_new = True
        if (dissimilarities and
            (min(dissimilarities) < g_max_dissimilarity_for_same)):
            vertex_is_new = False
        if vertex_is_new:
            # Add vertex to map.
            map_action = ActOnMapRequest()
            map_action.action = map_action.PUSH_VERTEX
            response = self.map_agent(map_action)
            new_vertex = response.objects[0].id
            debug('new vertex {}'.format(new_vertex))
            # Assign descriptors.
            map_action = ActOnMapRequest()
            map_action.object.id = new_vertex
            map_action.action = (
                map_action.ASSIGN_DESCRIPTOR_VERTEX)
            for link in descriptor_links:
                map_action.descriptor_id = link.descriptor_id
                map_action.interface_name = link.interface_name
                self.map_agent(map_action)
                if link.interface_name == self.crossing_interface_name:
                    self.current_crossing_id = link.descriptor_id
            # Get the exit_angles from map.
            # TODO: Don't use magic numbers (modify and rename
            # get_crossing_desc_id to return (desc, desc_id).
            # Crossing is the 2nd descriptor.
            crossing = self.crossing_getter(descriptor_links[1].descriptor_id)
            rospy.logdebug('Exit count: {}'.format(
                len(crossing.descriptor.frontiers)))
            # Add vertex and associate the sorted list of [None, angle].
            nodes = [[None, f.angle] for f in crossing.descriptor.frontiers]
            self.graph[new_vertex] = sorted(nodes)
            self.add_edge_to_graph(new_vertex)
            self.last_vertex = new_vertex
        else:
            index_vertex_same = dissimilarities.index(min(dissimilarities))
            vertex_same = vertices[index_vertex_same]
            self.add_edge_to_graph(vertex_same)
            self.last_vertex = vertex_same
            self.current_crossing_id = self.get_crossing_desc_id(vertex_same)
        if not self.first_vertex:
            self.first_vertex = self.last_vertex

    def get_dissimilarity(self):
        loc_goal = LocalizeGoal()
        loc_goal.action = loc_goal.GET_DISSIMILARITY
        debug('Requested GET_DISSIMILARITY')
        self.localize.send_goal_and_wait(loc_goal, rospy.Duration(0.5))
        loc_result = self.localize.get_result()
        if not loc_result:
            rospy.logerr('Did not received vertex descriptor within ' +
                         '0.5 s, exiting')
            return None, None
        debug('received {} dissimilarities'.format(len(loc_result.idata)))
        return loc_result.idata, loc_result.fdata

    def add_edge_to_graph(self, vertex):
        """Add an edge from self.last_vertex to vertex

        This means replace None with vertex for the adjacent vertex of
        self.last_vertex for which the edge information is self.exit_taken.
        """
        # TODO: rebuild the graph from the map...
        # so that several robots can be used simultaneously.
        if not self.last_vertex:
            return
        old_nodes = self.graph[self.last_vertex]
        new_nodes = []
        for v, a in old_nodes:
            if a == self.exit_taken:
                new_nodes.append([vertex, a])
                self.add_edge_to_map(self.last_vertex, vertex, self.exit_taken)
            else:
                new_nodes.append([v, a])
        self.graph[self.last_vertex] = new_nodes

    def add_edge_to_map(self, v0, v1, exit_angle):
        """Add an edge and its associated descriptor to the map

        The oriented edge is from v0 to v1.
        The edge descriptor is the exit angle to take at v0 to go to v1.
        """
        # Add edge.
        debug('adding edge ({}, {})'.format(
            rospy.get_name(), v0, v1))
        map_action = ActOnMapRequest()
        map_action.action = map_action.PUSH_EDGE
        map_action.object.type = map_action.object.EDGE
        map_action.object.references.append(v0)
        map_action.object.references.append(v1)
        edge_response = self.map_agent(map_action)
        debug('edge {} added'.format(edge_response.id))
        # Add descriptor.
        debug('adding descriptor')
        desc_response = self.exit_angles_setter(exit_angle)
        debug('descriptor {} added'.format(desc_response.id))
        # Assign descriptor.
        debug('assigining descriptor {} to edge {}'.format(
            desc_response.id, edge_response.id))
        map_action = ActOnMapRequest()
        map_action.action = map_action.ASSIGN_DESCRIPTOR_EDGE
        map_action.object.id = edge_response.id
        map_action.descriptor_id = desc_response.id
        self.map_agent(map_action)
        debug('descriptor assigned')

    def get_next_vertex_to_visit(self):
        """Return the tuple (vertex, angle)
        """
        def next_known(nodes):
            for v, _ in nodes:
                if (v is not None) and (v not in discovered):
                    return v
            return None

        def first_unknown(nodes):
            for v, a in nodes:
                if v is None:
                    return v, a
            return None

        rospy.logdebug('graph: {}'.format(self.graph))
        stack = [self.first_vertex]
        discovered = []
        while stack:
            nodes = self.graph[stack[-1]]
            v = next_known(nodes)
            if v is None:
                if first_unknown(nodes) is None:
                    discovered.append(stack.pop())
                else:
                    return first_unknown(nodes)
        return None

    def move_to_next_crossing(self):
        if self.next_vertex is None:
            return
        # TODO: remove angle from find_path output
        path = self.find_path()
        for vertex, angle in path:
            # Escape from crossing center.
            goal = NavigateGoal()
            goal.action = goal.TRAVERSE
            goal.edge.type = goal.edge.EDGE
            goal.edge.id = self.edge_id(self.last_vertex, vertex)
            if goal.edge.id is None:
                err = 'No edge from {} to {}'.format(self.last_vertex, vertex)
                rospy.logfatal(err)
                return False
            debug('escaping along edge {}'.format(
                goal.edge.id))
            result = self.escape.send_goal_and_wait(goal)
            if result.final_state != result.DONE:
                err = 'Escape jockey did not succeed'
                rospy.logerr(err)
                return False
            # Go to next crossing.
            goal = NavigateGoal()
            goal.action = goal.TRAVERSE
            debug('moving to next crossing')
            result = self.navigate.send_goal_and_wait(goal)
            if result.final_state != result.DONE:
                err = 'Escape jockey did not succeed'
                rospy.logerr(err)
                return False
            self.last_vertex = vertex
            self.exit_taken = angle
        return True

    def edge_id(self, v0, v1):
        """Return the id of edge from v0 to v1"""
        return get_edge_with_vertices(v0, v1).id

    def find_path(self):
        """Return a list of (vertex, angle)

        self.last_vertex (the crossing the robot presently is in) will not be
        part of the path. The last vertex will be self.next_vertex.
        """
        # TODO: remove angle from find_path output
        def adjacent_vertices(vertex):
            vertices = []
            for v, a in self.graph[vertex]:
                vertices.append(v)

        def path_edges(edges, end):
            edges = path_edges(edges, end)
            for v1, v0 in edges:
                if v1 == end:
                    return path(edges, v1) + [v0]

        def transform_graph(ingraph):
            graph = {}
            for k, node in ingraph.iteritems():
                for v, a in node:
                    graph[(k, v)] = a
            return graph

        def path(edges, end):
            edges = path_edges(edges, end)
            graph = transform_graph(self.graph)
            v0 = edges.pop()
            p = []
            while edges:
                v1 = edges.pop()
                p.append((v1, graph[(v0, v1)]))
            return p

        # TODO: use a BrowseVertex that has as attributes the BrowseVertex
        # it comes from during browsing and the associated angle so that
        # after the dfs-tree is built, the path can be found by browsing the
        # referenced vertices.
        print('Start vertex: '.format(self.last_vertex)) # DEBUG
        start = self.last_vertex
        print('End vertex: '.format(self.next_vertex)) # DEBUG
        end = self.next_vertex
        queue = [start]
        discovered = set()
        discovered.add(start)
        edges = []
        while queue:
            vertex = queue.pop(0)
            if vertex == end:
                return path(edges, end)
            for v in adjacent_vertices(vertex):
                if v not in discovered:
                    discovered.add(v)
                    edges.add((v, vertex))
                    queue.append(v)
        return None

    def get_crossing_desc_id(self, vertex):
        """Return the first Crossing descriptor associated with a vertex"""
        map_action = ActOnMapRequest()
        map_action.action = map_action.GET_DESCRIPTOR_LINKS
        map_action.object.id = vertex
        map_action.interface_name = self.crossing_interface_name
        response = self.map_agent(map_action)
        return response.descriptor_links[0].descriptor_id

    def escape_from_crossing(self):
        """Escape from crossing towards an unknown edge and return when done"""
        self.exit_angle_publisher.publish(self.next_exit)
        nav_goal = NavigateGoal()
        nav_goal.action = nav_goal.TRAVERSE
        nav_goal.descriptor_link.descriptor_id = self.current_crossing_id
        debug('escaping from crossing {} with direction {}'.format(
            nav_goal.descriptor_link.descriptor_id, self.next_exit))
        self.escape.send_goal_and_wait(nav_goal)
        escape_result = self.escape.get_result()
        if escape_result != escape_result.DONE:
            err = 'Escape jockey did not succeed'
            rospy.logerr(err)
            raise Exception(err)

node = ExplorerNode()
node.move_to_crossing()
node.loop()
