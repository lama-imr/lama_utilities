# -*- coding: utf-8 -*-

from math import pi

from lama_interfaces.graph_builder import get_descriptors
from lama_interfaces.graph_builder import get_directed_graph


def _angles_equal(a1, a2, precision=1e-5):
    """Return True is angle are considered equal

    Both angles should be within [-pi, pi].
    """
    return (abs(a1 - a2) < precision or
            abs(a1 - a2 + 2 * pi) < precision or
            abs(a1 - a2 - 2 * pi) < precision)


class GraphTransformer(object):
    """Read the map and produce a graph suitable for ExplorerNode

    The graph is organized as a map
    vertex: [[vertex, exit_angle], [vertex, exit_angle], ...].
    Where the second vertex is the vertex that will be at the next
    crossing center when traversing edge (corridor) at absolute angle
    exit_angle.
    When starting to traverse an edge, vertex is set to None. A vertex
    will be visited when all its neighbor vertices are not None.
    The graph is then an oriented graph where the information for edge
    a to b is the exit angle that was taken from a to reach b.
    """
    def __init__(self, map_agent,
                 crossing_getter, crossing_interface,
                 exit_angle_getter, exit_angle_interface):
        """
        Parameters
        ----------
        - map_agent: the ROS ServiceProxy for interactions with the map.
        - crossing_getter: ROS ServiceProxy for crossings.
        - crossing_interface: str, interface name for Crossing messages.
        - exit_angle_getter: ROS ServiceProxy for exit_angles.
        - exit_angle_interface: str, interface name for exit_angles.
        """
        self.map_agent = map_agent
        self.crossing_getter = crossing_getter
        self.crossing_interface = crossing_interface
        self.exit_angle_getter = exit_angle_getter
        self.exit_angle_interface = exit_angle_interface

    def graph_from_map(self):
        """Read the graph from the map

        Read the graph from the map and return a graph suitable for ExplorerNode
        """
        map_graph = get_directed_graph()
        graph = {}
        # Fill the graph keys (vertices) and values
        # (pairs [None|vertex, exit_angle]).
        for vertex, edges in map_graph.iteritems():
            key = vertex.id
            crossing = self.get_crossing(key)
            if crossing is None:
                continue
            graph[key] = []
            for frontier in crossing.frontiers:
                # TODO: optimize database retrieval of the same objects.
                end_vertex = self._get_end_vertex(edges,
                                                  frontier.angle)
                graph[key].append([end_vertex, frontier.angle])
        return graph

    def get_crossing(self, object_id):
        """Retrieve the crossing associated with a LamaObject

        Parameters
        ----------
        - object_id: int, LamaObject's id
        """
        crossings = get_descriptors(object_id,
                                    self.crossing_interface,
                                    self.crossing_getter)
        if crossings:
            return crossings[0]
        return

    def get_exit_angle(self, object_id):
        """Retrieve the exit_angle associated with a LamaObject

        Parameters
        ----------
        - object_id: int, LamaObject's id
        """
        return get_descriptors(object_id,
                               self.exit_angle_interface,
                               self.exit_angle_getter)[0]

    def _get_end_vertex(self, edges, angle):
        """Return the id of the end vertex of the edge with a given exit angle

        Parameters
        ----------
        - edges: list of edges (LamaObject).
        - angle: float, exit angle. The returned vertex is the one for which
            the edge corresponds to the crossing frontier with angle angle.
        """
        for edge in edges:
            edge_exit_angle = self.get_exit_angle(edge.id)
            if _angles_equal(edge_exit_angle, angle):
                return edge.references[1]
        return None
