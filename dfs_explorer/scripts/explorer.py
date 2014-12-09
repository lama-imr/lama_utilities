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
from lama_interfaces.core_interface import MapAgentInterface
# import lama_interfaces.cleartext_interface_factory as li_cif
# interface_factory = li_cif.cleartext_interface_factory
import lama_interfaces.interface_factory as li_if
interface_factory = li_if.interface_factory
from lama_interfaces.graph_builder import get_edges_with_vertices
from lama_interfaces.graph_builder import get_directed_graph_index

from graph_transformer import GraphTransformer

_max_dissimilarity_for_same = 0.05


def normalize_angles(angles):
    def normalize_angle(angle):
        return (angle + pi) % (2 * pi) - pi
    return [normalize_angle(a) for a in angles]


def debug(msg):
    rospy.logdebug('{}: {}'.format(rospy.get_name(), msg))


def jockey_client(jockey_name, action_type):
    client = actionlib.SimpleActionClient(jockey_name, action_type)
    while not client.wait_for_server(rospy.Duration(5)):
        rospy.loginfo(('{}: waiting for the jockey action ' +
                      'server ({})').format(rospy.get_name(), jockey_name))
    debug('communicating with the jockey ' +
          'action server {}'.format(jockey_name))
    return client


class Edge:
    def __init__(self, id_, preceeding):
        """An edge with link to its predecessor

        Parameters
        ----------
        id_: vertex id
        preceeding: Edge instance of preceeding edge
        """
        self.id = id_
        self.preceeding = preceeding

    def path_to_start(self):
        if self.preceeding is None:
            # We return an empty list because the start node should not be
            # part of the final path.
            return []
        return self.preceeding.path_to_start() + [self.id]


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
        iface = MapAgentInterface(start=False)
        self.map_agent = rospy.ServiceProxy(iface.action_service_name,
                                            ActOnMap)

        # Descriptor getter for Crossing.
        self.crossing_interface_name = rospy.get_param('~crossing_interface',
                                                       'crossing')
        crossing_getter_name = self.crossing_interface_name + '_getter'
        self.crossing_getter = rospy.ServiceProxy(crossing_getter_name,
                                                  GetCrossing)
        debug('waiting for service {}'.format(crossing_getter_name))
        self.crossing_getter.wait_for_service()
        debug('service {} available'.format(crossing_getter_name))

        # Exit angles getter and setter (double).
        self.exit_angles_interface_name = rospy.get_param(
            '~exit_angles_interface_name',
            'dfs_explorer_exit_angle')
        exits_iface = interface_factory(
            self.exit_angles_interface_name,
            'lama_interfaces/GetDouble',
            'lama_interfaces/SetDouble')
        self.exit_angles_getter = exits_iface.getter_service_proxy
        self.exit_angles_setter = exits_iface.setter_service_proxy

        # Exit angle topic advertiser.
        self.exit_angle_topic_name = rospy.get_param("exit_angle_topic",
                                                     "~exit_angle")

        self.first_crossing_reached = False
        # Last visited node, also current node if the robot is on a node.
        self.last_vertex = None
        # Exit taken, represented by its angle, when leaving the last node.
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
        self.graph_transformer = GraphTransformer(
            self.map_agent,
            self.crossing_getter,
            self.crossing_interface_name,
            self.exit_angles_getter,
            self.exit_angles_interface_name)

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
            # 1. Get a new vertex descriptor (robot should be at crossing
            #    center).
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

        Get the descriptors (i.e. write them into the database).
        Push the vertex if not already existent.
        Assign the descriptors to this vertex.

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
            rospy.logerr('Did not receive vertex descriptor within ' +
                         '0.5 s, exiting')
            return False
        rospy.logdebug('Received {} vertex descriptors'.format(
            len(loc_result.descriptor_links)))
        self.handle_vertex(loc_result.descriptor_links)
        return True

    def handle_vertex(self, descriptor_links):
        """Push a new vertex into the database, if needed

        Get the dissimilarity of the current descriptor (unsaved) with
          descriptors saved in the database.
        Save the vertex and associate the given descriptors, if the vertex is
          new.
        """
        vertex_come_from = self.last_vertex
        vertices, dissimilarities = self.get_dissimilarity()
        vertex_is_new = True
        if (dissimilarities and
            (min(dissimilarities) < _max_dissimilarity_for_same)):
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
            map_action.action = map_action.ASSIGN_DESCRIPTOR_VERTEX
            for link in descriptor_links:
                map_action.descriptor_id = link.descriptor_id
                map_action.interface_name = link.interface_name
                self.map_agent(map_action)
                if link.interface_name == self.crossing_interface_name:
                    self.current_crossing_id = link.descriptor_id
            # Get the exit_angles from the map.
            crossing_resp = self.crossing_getter(self.current_crossing_id)
            rospy.logdebug('Exit count: {}'.format(
                len(crossing_resp.descriptor.frontiers)))
            self.last_vertex = new_vertex
        else:
            # TODO: delete the redundant descriptors (quality-based
            # if possible)
            index_vertex_same = dissimilarities.index(min(dissimilarities))
            vertex_same = vertices[index_vertex_same]
            debug('already known vertex: {}'.format(vertex_same))
            self.last_vertex = vertex_same
            self.current_crossing_id = self.get_crossing_desc_id(vertex_same)
        # Add the edge to the map.
        if vertex_come_from is not None:
            self.add_edge_to_map(vertex_come_from, self.last_vertex,
                                 self.exit_taken)

    def get_dissimilarity(self):
        """Return two lists: indexes and dissimilarities"""
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

    def add_edge_to_map(self, v0, v1, exit_angle):
        """Add an edge and its associated descriptor to the map

        The oriented edge is from v0 to v1.
        The edge descriptor is the exit angle to take at v0 to go to v1.
        """
        # Add edge.
        debug('adding edge ({}, {})'.format(v0, v1))
        map_action = ActOnMapRequest()
        map_action.action = map_action.PUSH_EDGE
        map_action.object.type = map_action.object.EDGE
        map_action.object.references.append(v0)
        map_action.object.references.append(v1)
        edge_response = self.map_agent(map_action)
        if not edge_response.objects:
            rospy.logerr('Database error')
            return
        edge_id = edge_response.objects[0].id
        debug('edge {} ({} -> {}) added'.format(edge_id, v0, v1))
        # Add descriptor.
        debug('adding descriptor')
        desc_response = self.exit_angles_setter(exit_angle)
        debug('descriptor {} added, angle: {}'.format(desc_response.id,
                                                      exit_angle))
        # Assign descriptor.
        debug('assigining exit_angle descriptor {} to edge {}'.format(
            desc_response.id, edge_id))
        map_action = ActOnMapRequest()
        map_action.action = map_action.ASSIGN_DESCRIPTOR_EDGE
        map_action.object.id = edge_id
        map_action.descriptor_id = desc_response.id
        map_action.interface_name = self.exit_angles_interface_name
        self.map_agent(map_action)
        debug('descriptor assigned')

    def get_next_vertex_to_visit(self):
        """Return the tuple (vertex, angle), which is the next unvisited vertex

        Return None if all vertices were visited.

        A vertex is unvisited if one of its exit_angles has no associated
        vertex.

        vertex is None if the robot never explored the exit with angle angle.
        """
        def is_discovered(nodes):
            for v, a in nodes:
                if v is None:
                    return False, (v, a)
            return True, (None, None)

        graph = self.graph_transformer.graph_from_map()
        stack = [(self.last_vertex, self.exit_taken)]
        discovered = []
        while stack:
            v = stack.pop(0)
            nodes = graph[v[0]]
            this_is_discovered, node = is_discovered(nodes)
            if this_is_discovered:
                if v not in discovered:
                    discovered.append(v)
                    for new_node in nodes:
                        stack.append(new_node)
            else:
                return (v[0], node[1])

    def move_to_next_crossing(self):
        if self.next_vertex is None:
            # The current vertex is unvisited, don't need to move to another
            # vertex.
            return
        path = self.find_path_to_next_vertex()
        for vertex in path:
            # Escape from crossing center.
            goal = NavigateGoal()
            goal.action = goal.TRAVERSE
            goal.edge = self.first_edge(self.last_vertex, vertex)
            if goal.edge.id is None:
                err = 'No edge from {} to {}'.format(self.last_vertex, vertex)
                rospy.logfatal(err)
                return False
            debug('escaping along edge {}'.format(goal.edge.id))
            self.escape.send_goal_and_wait(goal)
            result = self.navigate.get_result()
            if result.final_state != result.DONE:
                err = 'Escape jockey did not succeed'
                rospy.logerr(err)
                return False
            # Go to next crossing.
            goal = NavigateGoal()
            goal.action = goal.TRAVERSE
            debug('moving to next crossing')
            self.navigate.send_goal_and_wait(goal)
            result = self.navigate.get_result()
            if result.final_state != result.DONE:
                err = 'Escape jockey did not succeed'
                rospy.logerr(err)
                return False
            self.last_vertex = vertex
        return True

    def first_edge(self, v0, v1):
        """Return the first found edge (LamaObject) from v0 to v1"""
        edges = get_edges_with_vertices(v0, v1)
        if edges:
            return edges[0]
        return None

    def find_path_to_next_vertex(self):
        """Return a list of vertices defining a path to self.next_vertex

        Return a list of vertices defining a path from the robot current
        position to self.next_vertex.
        self.last_vertex (the crossing the robot presently is in) will not be
        part of the path. The last vertex will be self.next_vertex.
        """
        graph = get_directed_graph_index()
        start = self.last_vertex
        end = self.next_vertex
        queue = [Edge(start, None)]
        discovered = set()
        while queue:
            edge = queue.pop(0)
            vertex = edge.id
            if vertex == end:
                rospy.loginfo('Found path (from {}): {}'.format(
                    start, edge.path_to_start()))
                return edge.path_to_start()
            if vertex not in discovered:
                discovered.add(vertex)
                for adjacent_vertex in graph[vertex]:
                    queue.append(Edge(adjacent_vertex, edge))
        return None

    def get_crossing_desc_id(self, vertex):
        """Return the id of the first Crossing associated with a vertex"""
        map_action = ActOnMapRequest()
        map_action.action = map_action.GET_DESCRIPTOR_LINKS
        map_action.object.id = vertex
        map_action.interface_name = self.crossing_interface_name
        response = self.map_agent(map_action)
        return response.descriptor_links[0].descriptor_id

    def escape_from_crossing(self):
        """Escape from crossing towards an unknown edge and return when done"""
        exit_angle_publisher = rospy.Publisher(self.exit_angle_topic_name,
                                               Float32,
                                               queue_size=1,
                                               latch=True)
        exit_angle_publisher.publish(self.next_exit)
        nav_goal = NavigateGoal()
        nav_goal.action = nav_goal.TRAVERSE
        nav_goal.descriptor_link.descriptor_id = self.current_crossing_id
        debug('escaping from crossing {} with direction {}'.format(
            nav_goal.descriptor_link.descriptor_id, self.next_exit))
        self.escape.send_goal_and_wait(nav_goal)
        escape_result = self.escape.get_result()
        if escape_result.final_state != escape_result.DONE:
            err = 'Escape jockey did not succeed'
            rospy.logerr(err)
            raise Exception(err)
        self.exit_taken = self.next_exit

node = ExplorerNode()
node.move_to_crossing()
node.loop()
