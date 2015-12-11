#!/usr/bin/python
# -*- coding: utf-8 -*-

# Implement a Depth-First Search explorer.
# Works in combination with
# a) lj_laser_heading, nj_laser, and nj_escape_crossing
# or
# b) lj_costmap, nj_costmap, and nj_escape_crossing.

from math import pi

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose

from lama_interfaces.srv import ActOnMapRequest
from lama_interfaces.map_agent import MapAgent
from lama_jockeys.msg import NavigateAction
from lama_jockeys.msg import NavigateGoal
from lama_jockeys.msg import LocalizeAction
from lama_jockeys.msg import LocalizeGoal
from lama_msgs.msg import LamaObject
from lama_msgs.srv import GetCrossing
from lama_interfaces.graph_builder import get_edges_with_vertices
from lama_interfaces.graph_builder import get_directed_graph
from lama_interfaces.graph_builder import get_directed_graph_index
from lama_interfaces.graph_builder import get_vertex_from_graph
from lama_interfaces.interface_factory import interface_factory


def normalize_angles(angles):
    def normalize_angle(angle):
        return (angle + pi) % (2 * pi) - pi
    return [normalize_angle(a) for a in angles]


def jockey_client(jockey_name, action_type):
    client = actionlib.SimpleActionClient(jockey_name, action_type)
    while not client.wait_for_server(rospy.Duration(5)):
        rospy.loginfo(('{}: waiting for the jockey action ' +
                      'server ({})').format(rospy.get_name(), jockey_name))
        if rospy.is_shutdown():
            return
    rospy.logdebug('communicating with the jockey ' +
                   'action server {}'.format(jockey_name))
    return client


def service_client(service_name, service_type):
    client = rospy.ServiceProxy(service_name, service_type)
    client_ok = False
    while not client_ok:
        if rospy.is_shutdown():
            return
        try:
            client.wait_for_service(5)
        except rospy.ROSException:
            rospy.logdebug('waiting for service "{}"'.format(service_name))
        else:
            client_ok = True
    rospy.logdebug('service "{}" available'.format(service_name))
    return client


class Edge:
    def __init__(self, id_, preceeding):
        """An edge with reference to its predecessor

        Parameters
        ----------
        id_: start vertex id
        preceeding: Edge instance of preceeding edge
        """
        self.id = id_
        self.preceeding = preceeding

    def path_to_start(self):
        if self.preceeding is None:
            # We return the current node because the start node is part of the
            # final path.
            return [self.id]
        return self.preceeding.path_to_start() + [self.id]


class ExplorerNode(object):
    def __init__(self):
        # Node initialization and parameter interface.
        rospy.init_node('dfs_explorer')
        navigating_jockey_name = rospy.get_param('~navigating_jockey_name',
                                                 'navigating_jockey')
        localizing_jockey_name = rospy.get_param('~localizing_jockey_name',
                                                 'localizing_jockey')
        escape_jockey_name = rospy.get_param('~escape_jockey_name',
                                             'nj_escape_jockey')
        self.crossing_interface_name = rospy.get_param('~crossing_interface',
                                                       'crossing')
        self.place_profile_interface_name = rospy.get_param(
            '~place_profile_interface',
            'place_profile')
        self.exit_angles_interface_name = rospy.get_param(
            '~exit_angles_interface_name',
            'dfs_explorer_exit_angle')
        self.dissimilarity_threshold = rospy.get_param(
            '~dissimilarity_threshold',
            0.05)

        self.initialized = False

        # Navigate jockey server.
        self.navigate = jockey_client(navigating_jockey_name, NavigateAction)
        if not self.navigate:
            return

        # Localize jockey server.
        self.localize = jockey_client(localizing_jockey_name, LocalizeAction)
        if not self.localize:
            return

        # Crossing escape jockey server.
        self.escape = jockey_client(escape_jockey_name, NavigateAction)
        if not self.escape:
            return

        # Map agent client.
        self.map_agent = MapAgent(timeout=5)
        if not self.map_agent.wait_for_service():
            return
        # We know the service is ready, don't wait any more.
        self.map_agent.timeout = None

        # Descriptor getter for Crossing.
        crossing_getter_name = self.crossing_interface_name + '_getter'
        self.crossing_getter = service_client(crossing_getter_name,
                                              GetCrossing)
        if not self.crossing_getter:
            return

        # Exit angles getter and setter (double).
        exits_iface = interface_factory(
            self.exit_angles_interface_name,
            'lama_interfaces/GetDouble',
            'lama_interfaces/SetDouble')
        self.exit_angles_getter = exits_iface.getter_service_proxy
        self.exit_angles_setter = exits_iface.setter_service_proxy

        self.first_crossing_reached = False
        # When leaving a vertex, this is be the next edge to visit.
        # When arriving at a vertex, this is the last traversed edge.
        self.current_edge = None
        # The next edge to visit in the DFS algorithm.
        self.edge_to_visit = None

        rospy.logdebug('initialized')
        self.initialized = True

    def move_to_crossing(self):
        """Move the robot to the next encountered crossing"""
        rospy.logdebug('moving to crossing')
        nav_goal = NavigateGoal()
        nav_goal.action = nav_goal.TRAVERSE
        self.navigate.send_goal(nav_goal)
        self.navigate.wait_for_result()
        nav_result = self.navigate.get_result()
        if nav_result and nav_result.final_state == nav_result.DONE:
            rospy.logdebug(('traversed to crossing center in {:.2f} s').format(
                nav_result.completion_time.to_sec()))
        else:
            err = ('Something wrong happened while getting TRAVERSE' +
                   ' result, exiting!')
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
        if not self.initialized:
            rospy.logerr('instance not initialized')
            raise Exception('instance not initialized')
        if not self.first_crossing_reached:
            rospy.logerr('Go to first crossing first')
            raise Exception('Go to first crossing first')

        while not rospy.is_shutdown():
            # 1. Get a new vertex descriptor (robot should be at crossing
            #    center).
            rospy.logdebug('getting descriptor')
            if not self.get_current_descriptor():
                rospy.logwarn('No descriptor, exiting')
                break

            # 2. Choose the next edge to visit.
            self.edge_to_visit = self.get_next_edge_to_visit(
                self.current_edge.references[1])
            if self.edge_to_visit is None:
                rospy.loginfo('I visisted all crossings, successfully exiting')
                break
            rospy.logdebug('next edge to visit {}'.format(
                self.edge_to_visit.id))

            # 3. Move to the start vertex of that edge.
            current_vertex = self.current_edge.references[1]
            self.move_to_next_crossing(current_vertex, self.edge_to_visit)
            self.current_edge = self.edge_to_visit

            # 4. Let the robot escape from the node along the chosen
            # unexplored edge.
            self.escape_from_crossing(self.edge_to_visit)

            # 5. Let the navigating jockey move to the next crossing.
            self.move_to_crossing()

    def get_current_descriptor(self):
        """Get the descriptors from the current crossing center

        The robot is assumed to be at a crossing center.
        Get the descriptors (i.e. have them written into the database).
        Push the vertex if not already existing.
        Assign the descriptors to this vertex.

        Parameters
        ----------
        - origin_angle: float, absolute angle of the exit the robot took
            when starting from origin_vertex.
        """
        loc_goal = LocalizeGoal()
        loc_goal.action = loc_goal.GET_VERTEX_DESCRIPTOR
        state = self.localize.send_goal_and_wait(loc_goal)
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Did not receive vertex descriptor, exiting')
            return False
        loc_result = self.localize.get_result()
        if not loc_result:
            rospy.logerr('Goal succeeded but result not set, exiting')
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
        vertices, dissimilarities = self.get_dissimilarity()
        if vertices:
            rospy.logdebug('vertices, dissimilarities: {}'.format(
                zip(vertices, dissimilarities)))
        vertex_is_new = True
        if (dissimilarities and
            (min(dissimilarities) < self.dissimilarity_threshold)):
            vertex_is_new = False
        if vertex_is_new:
            # Add vertex to map.
            response = self.map_agent.proxy(action=ActOnMapRequest.PUSH_VERTEX)
            if not response:
                rospy.logerr('Database error')
            new_vertex = response.objects[0]
            current_vertex_id = new_vertex.id
            rospy.logdebug('new vertex {}'.format(current_vertex_id))
            # Assign descriptors.
            map_action = ActOnMapRequest()
            map_action.object = new_vertex
            map_action.action = map_action.ASSIGN_DESCRIPTOR_VERTEX
            for link in descriptor_links:
                map_action.descriptor_id = link.descriptor_id
                map_action.interface_name = link.interface_name
                response = self.map_agent.proxy(map_action)
                if not response:
                    rospy.logerr('Database error')
                if link.interface_name == self.crossing_interface_name:
                    current_crossing_id = link.descriptor_id
            # Get the exit_angles from the map.
            crossing_resp = self.crossing_getter(current_crossing_id)
            rospy.logdebug('Exit count: {}'.format(
                len(crossing_resp.descriptor.frontiers)))
            # Add the edges from this vertex to the unknown vertex (-1).
            for frontier in crossing_resp.descriptor.frontiers:
                self.add_edge_to_map(current_vertex_id, -1, frontier.angle)
        else:
            # TODO: delete the redundant descriptors (quality-based
            # if possible)
            index_vertex_same = dissimilarities.index(min(dissimilarities))
            current_vertex_id = vertices[index_vertex_same]
            rospy.logdebug('already known vertex: {}'.format(current_vertex_id))

        # Modify the edge we just traversed or fake it, the first time.
        if self.current_edge:
            # Modify an edge already in the map.
            self.change_edge_end(self.current_edge.id, current_vertex_id)
        else:
            # We did not traverse any edge the first time we arrive at a
            # crossing but we set an end vertex for self.get_next_edge_to_visit.
            self.current_edge = LamaObject()
        self.current_edge.references[1] = current_vertex_id

    def get_dissimilarity(self):
        """Return two lists: indexes and dissimilarities"""
        loc_goal = LocalizeGoal()
        loc_goal.action = loc_goal.GET_DISSIMILARITY
        rospy.logdebug('Requested GET_DISSIMILARITY')
        state = self.localize.send_goal_and_wait(loc_goal)
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Did not receive dissimilarities, exiting')
            return None, None
        loc_result = self.localize.get_result()
        if not loc_result:
            rospy.logerr('Goal succeeded but result not set, exiting')
            return None, None
        rospy.logdebug(
            'received {} dissimilarities'.format(len(loc_result.idata)))
        return loc_result.idata, loc_result.fdata

    def add_edge_to_map(self, v0, v1, exit_angle):
        """Add an edge and its associated descriptor to the map

        The oriented edge is from v0 to v1.
        The edge descriptor is the exit angle to take at v0 to go to v1.
        """
        # Add edge.
        rospy.logdebug('adding edge ({}, {})'.format(v0, v1))
        map_action = ActOnMapRequest()
        map_action.action = map_action.PUSH_EDGE
        map_action.object.type = map_action.object.EDGE
        map_action.object.references[0] = v0
        map_action.object.references[1] = v1
        edge_response = self.map_agent.proxy(map_action)
        if not edge_response.objects:
            rospy.logerr('Database error')
            return
        edge_id = edge_response.objects[0].id
        rospy.logdebug('edge {} ({} -> {}) added'.format(edge_id, v0, v1))
        # Add descriptor.
        rospy.logdebug('adding descriptor')
        desc_response = self.exit_angles_setter(exit_angle)
        if not desc_response:
            rospy.logerror('Error on call to {}'.format(self.exit_angles_setter.resolved_name))
            return
        rospy.logdebug('descriptor {} added, angle: {}'.format(desc_response.id,
                                                               exit_angle))
        # Assign descriptor.
        rospy.logdebug('assigning exit_angle descriptor {} to edge {}'.format(
            desc_response.id, edge_id))
        map_action = ActOnMapRequest()
        map_action.action = map_action.ASSIGN_DESCRIPTOR_EDGE
        map_action.object.id = edge_id
        map_action.descriptor_id = desc_response.id
        map_action.interface_name = self.exit_angles_interface_name
        self.map_agent.proxy(map_action)
        rospy.logdebug('descriptor assigned')

    def change_edge_end(self, edge_id, v1):
        """Modify the end vertex of an edge"""
        map_action = ActOnMapRequest()
        map_action.action = map_action.PUSH_EDGE
        map_action.object.id = edge_id
        map_action.object.references[1] = v1
        edge_response = self.map_agent.proxy(map_action)
        if not edge_response.objects:
            rospy.logerr('Database error')
            return

    def get_next_edge_to_visit(self, current_vertex):
        """Return the next unvisited edge as LamaObject

        Return None if all edges were visited.

        An edge is unvisited if it end vertex is -1.
        """
        def edge_to_discover(outgoing_edges):
            """Return (False, edge) or (True, None)"""
            for e in outgoing_edges:
                if e.references[1] == -1:
                    return e
            return

        graph = get_directed_graph()
        stack = [get_vertex_from_graph(graph, current_vertex)]
        discovered = []
        while stack:
            v = stack.pop(0)
            edges = graph[v]
            edge = edge_to_discover(edges)
            if edge is None:
                if v not in discovered:
                    discovered.append(v)
                    for edge in edges:
                        stack.append(get_vertex_from_graph(graph,
                                                           edge.references[1]))
            else:
                return edge

    def move_to_next_crossing(self, current_vertex, edge_to_visit):
        target_vertex = edge_to_visit.references[0]
        if current_vertex == target_vertex:
            # The current vertex is the start vertex of the edge we want to
            # reach, don't need to move.
            self.current_edge = edge_to_visit
            return
        path = self.find_path(current_vertex, target_vertex)
        if None in path:
            err = 'no path from {} to {}'.format(current_vertex,
                                                 target_vertex)
            rospy.logfatal(err)
            return False

        msg = 'Path: '
        for edge in path:
            msg += '{} --> '.format(edge.references[0])
        msg += '{}'.format(edge.references[1])
        rospy.logdebug(msg)

        for edge in path:
            self.escape_from_crossing(edge)

            # Go to next crossing.
            goal = NavigateGoal()
            goal.action = goal.TRAVERSE
            rospy.logdebug('moving to next crossing')
            self.navigate.send_goal_and_wait(goal)
            result = self.navigate.get_result()
            if not result or (result.final_state != result.DONE):
                err = 'Escape jockey did not succeed'
                rospy.logerr(err)
                return False
            self.current_edge = edge
        return True

    def first_edge(self, v0, v1):
        """Return the first found edge (LamaObject) from v0 to v1"""
        edges = get_edges_with_vertices(v0, v1)
        if edges:
            return edges[0]
        return None

    def find_path(self, start, end):
        """Return a list of edges defining a path from start to end vertices

        start and end are ids, not Lama Objects.
        Return a list of edges as Lama Objects.
        """
        def get_edge_path_from_vertex_path(vpath):
            epath = []
            for v0, v1 in zip(vpath[:-1], vpath[1:]):
                epath.append(self.first_edge(v0, v1))
            return epath

        graph = get_directed_graph_index()
        queue = [Edge(start, None)]
        discovered = set()
        while queue:
            edge = queue.pop(0)
            vertex = edge.id
            if vertex == end:
                rospy.loginfo('Found path (from {}): {}'.format(
                    start, edge.path_to_start()))
                return get_edge_path_from_vertex_path(edge.path_to_start())
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
        response = self.map_agent.proxy(map_action)
        if not response.descriptor_links:
            rospy.logerr('No Crossing (interface {}) associated with ' +
                         'vertex {}'.format(
                             self.crossing_interface_name, vertex.id))
            return None
        if len(response.descriptor_links) > 1:
            rospy.logwarn('More than one Crossing associated with ' +
                          'vertex {}, taking the first one'.format(vertex.id))
        return response.descriptor_links[0].descriptor_id

    def get_place_profile_desc_id(self, vertex):
        """Return the id of the first PlaceProfile associated with a vertex

        Parameters
        ----------
        - vertex: int, id of a vertex
        """
        map_action = ActOnMapRequest()
        map_action.action = map_action.GET_DESCRIPTOR_LINKS
        map_action.object.id = vertex
        map_action.interface_name = self.place_profile_interface_name
        response = self.map_agent.proxy(map_action)
        if not response.descriptor_links:
            rospy.logerr('No PlaceProfile (interface {}) associated with ' +
                         'vertex {}'.format(
                             self.place_profile_interface_name, vertex))
            return None
        if len(response.descriptor_links) > 1:
            rospy.logwarn('More than one PlaceProfile associated with ' +
                          'vertex {}, taking the first one'.format(vertex))
        return response.descriptor_links[0].descriptor_id

    def escape_from_crossing(self, edge_to_traverse):
        """Escape from crossing along edge_to_traverse (LamaObject)"""
        # Get the robot pose relative to the vertex.
        loc_goal = LocalizeGoal()
        loc_goal.action = loc_goal.LOCALIZE_IN_VERTEX
        loc_goal.descriptor_link.descriptor_id = (
            self.get_place_profile_desc_id(edge_to_traverse.references[0]))
        if not loc_goal.descriptor_link.descriptor_id:
            rospy.logerr('Database error')
            return False
        loc_goal.descriptor_link.interface_name = (
            self.place_profile_interface_name)
        self.localize.send_goal_and_wait(loc_goal)
        loc_result = self.localize.get_result()
        if not loc_result or (loc_result.state != loc_result.DONE):
            err = 'Localize jockey, LOCALIZE_IN_VERTEX did not succeed'
            rospy.logerr(err)
            return False
        pose = Pose()
        pose.position.x = loc_result.fdata[0]
        pose.position.y = loc_result.fdata[1]
        pose.position.z = loc_result.fdata[2]
        pose.orientation.x = loc_result.fdata[3]
        pose.orientation.y = loc_result.fdata[4]
        pose.orientation.z = loc_result.fdata[5]
        pose.orientation.w = loc_result.fdata[6]

        nav_goal = NavigateGoal()
        nav_goal.action = nav_goal.TRAVERSE
        nav_goal.edge = edge_to_traverse
        nav_goal.relative_edge_start = pose
        rospy.logdebug('Escaping from crossing along edge {}'.format(
            edge_to_traverse.id))
        self.escape.send_goal_and_wait(nav_goal)
        escape_result = self.escape.get_result()
        if ((not escape_result) or
            (escape_result.final_state != escape_result.DONE)):
            err = 'Escape jockey did not succeed'
            rospy.logerr(err)
            raise Exception(err)
        return True

node = ExplorerNode()
if node.initialized:
    node.move_to_crossing()
    node.loop()
