# Class for management of the map core functionalities.
# This includes management of vertices, edges, and descriptor links.

import copy

from sqlalchemy import select
from sqlalchemy import ForeignKey
from sqlalchemy import Table
from sqlalchemy import Column
from sqlalchemy import types

import rospy
import roslib.msgs
import roslib.message

from lama_interfaces.msg import LamaObject
from lama_interfaces.msg import DescriptorLink
from lama_interfaces.srv import ActOnMapResponse

from abstract_db_interface import AbstractDBInterface

g_default_core_table_name = 'lama_object'
g_default_descriptor_table_name = 'lama_descriptor_link'
g_default_map_agent_name = '/lama_map_agent'

# sqlalchemy engine (argument to sqlalchemy.create_engine)
g_engine_name = rospy.get_param('/database_engine', 'sqlite:///created.sqlite')


class CoreDBInterface(AbstractDBInterface):
    def __init__(self, engine, interface_name=None, descriptor_table_name=None,
                 start=False):
        """Build the map interface for LamaObject

        Parameters
        ----------
        - engine: String, argument to sqlalchemy.create_engine.
        - interface_name: string, name of the table containing LamaObject
            messages (vertices and edges). Defaults to 'lama_objects'.
        - descriptor_table_name: string, name of the table for DescriptorLink
            messages. Defaults to 'lama_descriptor_links'.
        """
        self._check_md5sum()
        if not interface_name:
            interface_name = g_default_core_table_name
        if not descriptor_table_name:
            descriptor_table_name = g_default_descriptor_table_name
        self.descriptor_table_name = descriptor_table_name

        get_srv_type = 'lama_interfaces/GetLamaObject'
        set_srv_type = 'lama_interfaces/SetLamaObject'
        super(CoreDBInterface, self).__init__(engine, interface_name,
                                              get_srv_type, set_srv_type,
                                              start=start)

    @property
    def interface_type(self):
        return 'core'

    def _check_md5sum(self):
        """Check that current implementation is compatible with LamaObject"""
        lama_object = LamaObject()
        if lama_object._md5sum != "c453f8236f4931ea54fb608514a25926":
            raise rospy.ROSException("CoreDBInterface incompatible " +
                                     "with current LamaObject implementation")

    def _generate_schema(self):
        """Create the SQL tables"""
        self._add_interface_description()
        self._generate_core_table()
        self._generate_descriptor_table()
        self.metadata.create_all()

    def _generate_core_table(self):
        """Create the SQL tables for LamaObject messages"""
        # The table format is hard-coded.
        table = Table(self.interface_name,
                      self.metadata,
                      Column('id', types.Integer, primary_key=True),
                      extend_existing=True)
        table.append_column(Column('id_in_world',
                                   types.Integer))
        table.append_column(Column('name', types.String))
        table.append_column(Column('type', types.Integer))
        self.core_table = table

        table = Table(self.interface_name + '@references@_data_',
                      self.metadata)
        table.append_column(Column('id', types.Integer, primary_key=True))
        table.append_column(Column('seq_num', types.Integer))
        table.append_column(Column('parent_id', types.Integer,
                                   ForeignKey(self.interface_name + '.id')))
        table.append_column(Column('_value_', types.Integer))
        self.core_obj_ref_table = table

    def _generate_descriptor_table(self):
        """Create the SQL tables for descriptor_links"""
        table = Table(self.descriptor_table_name,
                      self.metadata,
                      Column('id', types.Integer, primary_key=True),
                      extend_existing=True)
        table.append_column(Column('object_id',
                                   types.Integer,
                                   ForeignKey(self.interface_name + '.id')))
        table.append_column(Column('descriptor_id', types.Integer))
        table.append_column(Column('interface_name', types.String))
        table.append_column(Column('timestamp_secs', types.Integer))
        table.append_column(Column('timestamp_nsecs', types.Integer))
        # TODO: add a uniqueness constraint on (object_id, descriptor_id,
        # interface_name)
        self.descriptor_table = table

    def getter_callback(self, msg):
        """Get a LamaObject from the database

        Get a LamaObject from the database, from its id.
        Return an instance of GetLamaObject.srv response.

        Parameters
        ----------
        - msg: an instance of GetLamaObject.srv request.
        """
        id_ = msg.id
        lama_object = self.get_lama_object(id_)
        # Create an instance of getter response.
        response = self.getter_service_class._response_class()
        response.object = lama_object
        return response

    def setter_callback(self, msg):
        """Add a LamaObject message to the database

        Return an instance of SetLamaObject.srv response.

        Parameters
        ----------
        - msg: an instance of SetLamaObject.srv request.
        """
        # Create an instance of setter response.
        response = self.setter_service_class._response_class()
        response.id = self.set_lama_object(msg.object)
        return response

    def get_lama_object(self, id_):
        """Get a vertex or an edge from its unique id_

        Return an instance of LamaObject.

        Parameters
        ----------
        - id_: int, lama object id (id in the database).
        """
        # Make the transaction for the core table.
        query = self.core_table.select(
            whereclause=(self.core_table.c.id == id_))
        connection = self.engine.connect()
        transaction = connection.begin()
        result = connection.execute(query).fetchone()
        transaction.commit()
        connection.close()
        if not result:
            err = 'No element with id {} in database table {}'.format(
                id_, self.core_table.name)
            raise rospy.ServiceException(err)

        lama_object = LamaObject()
        lama_object.id = id_
        lama_object.id_in_world = result['id_in_world']
        lama_object.name = result['name']
        lama_object.type = result['type']

        if lama_object.type == LamaObject.EDGE:
            lama_object.references = self._get_lama_object_references(id_)
            if len(lama_object.references) != 2:
                raise rospy.ServiceException('Corrupted database')

        return lama_object

    def _get_lama_object_references(self, id_):
        """Get the references of a LamaObject"""
        # Make the transaction for the array 'references'.
        query = self.core_obj_ref_table.select(
            whereclause=(self.core_obj_ref_table.parent_id == id_))
        connection = self.engine.connect()
        transaction = connection.begin()
        refs = connection.execute(query).fetchall()
        transaction.commit()
        connection.close()
        if not refs:
            return []
        if len(refs) != 2:
            raise rospy.ServiceException('Corrupted database')
        references = [0] * 2
        for ref in refs:
            index = ref['seq_num']
            val = ref['_value_']
            references[index] = val
        return references

    def get_lama_objects(self, id_in_world):
        """Get a list of vertices or edges from their id_in_world

        Return a list of LamaObject instances.

        Parameters
        ----------
        - id_in_world: int, lama object's id_in_world (not id in the database).
        """
        # Make the transaction for the core table.
        connection = self.engine.connect()
        transaction = connection.begin()
        query = self.core_table.select(
            whereclause=(self.core_table.c.id_in_world == id_in_world))
        results = connection.execute(query).fetchall()
        transaction.commit()
        connection.close()
        if not results:
            err = 'No element with id_in_world {} in database table {}'.format(
                id_in_world, self.core_table.name)
            raise rospy.ServiceException(err)

        lama_objects = []
        for result in results:
            lama_object = LamaObject()
            lama_object.id = result['id']
            lama_object.id_in_world = id_in_world
            lama_object.name = result['name']
            lama_object.type = result['type']

            if lama_object.type == LamaObject.EDGE:
                lama_object.references = self._get_lama_object_references(
                    lama_object.id)
                if len(lama_object.references) != 2:
                    raise rospy.ServiceException('Corrupted database')
            lama_objects.append(lama_object)

        return lama_objects

    def get_descriptor_links(self, id_, interface_name=None):
        """Retrieve the list of DescriptorLink associated with a Lama object

        Return a list of DescriptorLink. If interface_name is given, return
        all DescriptorLink corresponding to this interface_name, otherwise, and
        if interface_name is '' or '*', return all DescriptorLink.

        Parameters
        ----------
        - id_: int, lama object id in the database.
        - interface_name: string, default to None.
            If None, '', or '*', all DescriptorLink are returned.
            Otherwise, only DescriptorLink from this interface are returned.
        """
        desc_links = []
        # Make the transaction from the descriptor table.
        table = self.descriptor_table
        query = table.select()
        query = query.where(table.c.object_id == id_)
        if interface_name and interface_name != '*':
            query = query.where(table.c.interface_name == interface_name)
        connection = self.engine.connect()
        transaction = connection.begin()
        results = connection.execute(query).fetchall()
        transaction.commit()
        connection.close()
        if not results:
            return []
        for result in results:
            desc_link = DescriptorLink()
            desc_link.object_id = id_
            desc_link.descriptor_id = result['descriptor_id']
            desc_link.interface_name = result['interface_name']
            desc_links.append(desc_link)
        return desc_links

    def set_lama_object(self, lama_object):
        """Add a lama object to the database

        Return the lama object's id.

        Parameter
        ---------
        - lama_object: an instance of LamaObject.
        """
        # Make the transaction for the core table.
        connection = self.engine.connect()
        transaction = connection.begin()
        insert_args = {
            'id_in_world': lama_object.id_in_world,
            'name': lama_object.name,
            'type': lama_object.type,
        }
        result = connection.execute(self.core_table.insert(), insert_args)
        object_id = result.inserted_primary_key[0]
        transaction.commit()
        connection.close()

        # Make the transaction for the 'references' array.
        if lama_object.type == LamaObject.EDGE:
            connection = self.engine.connect()
            transaction = connection.begin()
            for i, v in enumerate(lama_object.references):
                insert_args = {
                    'seq_num': i,
                    'parent_id': object_id,
                    '_value_': v,
                }
                connection.execute(self.core_obj_ref_table.insert(),
                                   insert_args)
            transaction.commit()
            connection.close()

        self._set_timestamp(rospy.Time.now())
        return object_id


class MapAgentInterface(object):
    """Define callbacks for ActOnMap and start the map agent service"""
    def __init__(self, start=False):
        action_srv_type = 'lama_interfaces/ActOnMap'
        srv_action_class = roslib.message.get_service_class(action_srv_type)
        # Action class.
        map_agent_name = rospy.get_param('map_agent', g_default_map_agent_name)
        self.action_service_name = map_agent_name
        self.action_service_class = srv_action_class
        if start:
            self.map_agent = rospy.Service(self.action_service_name,
                                           self.action_service_class,
                                           self.action_callback)
        else:
            self.map_agent = None

        self.core_iface = CoreDBInterface(g_engine_name, start=False)
        self.engine = self.core_iface.engine

    def action_callback(self, msg):
        """Callback of ActOnMap service

        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        callbacks = {
            msg.PUSH_VERTEX: self.push_lama_object,
            msg.PULL_VERTEX: self.pull_lama_object,
            msg.ASSIGN_DESCRIPTOR_VERTEX: (
                self.assign_descriptor_to_lama_object),
            msg.PUSH_EDGE: self.push_lama_object,
            msg.PULL_EDGE: self.pull_lama_object,
            msg.ASSIGN_DESCRIPTOR_EDGE: (
                self.assign_descriptor_to_lama_object),
            msg.GET_VERTEX_LIST: self.get_vertex_list,
            msg.GET_EDGE_LIST: self.get_edge_list,
            msg.GET_DESCRIPTOR_LINKS: self.get_descriptor_links,
            msg.GET_NEIGHBOR_VERTICES: self.get_neighbor_vertices,
            msg.GET_OUTGOING_EDGES: self.get_outgoing_edges,
        }
        if msg.action not in callbacks:
            raise rospy.ServiceException('Action {} not implemented'.format(
                msg.action))
        r = callbacks[msg.action](msg)
        return r

    def push_lama_object(self, msg):
        """Add a LaMa object to the database

        Callback for PUSH_VERTEX and PUSH_EDGE.
        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        lama_object = copy.copy(msg.object)
        lama_object.id = self.core_iface.set_lama_object(lama_object)
        response = ActOnMapResponse()
        response.objects.append(lama_object)
        return response

    def pull_lama_object(self, msg):
        """Retrieve a LaMa object from the database

        Callback for PULL_VERTEX and PULL_EDGE.
        Return an instance of ActOnMap response. The field descriptor_links will
        be filled with all DescriptorLink associated with this LamaObject.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        response = ActOnMapResponse()
        id_ = msg.object.id
        response.objects.append(self.core_iface.get_lama_object(id_))
        get_desc_links = self.core_iface.get_descriptor_links
        response.descriptor_links = get_desc_links(id_)
        return response

    def assign_descriptor_to_lama_object(self, msg):
        """Add a descriptor to a vertex

        Callback for ASSIGN_DESCRIPTOR_VERTEX and ASSIGN_DESCRIPTOR_EDGE.
        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        # Ensure that the lama object exists in the core table.
        object_id = msg.object.id
        core_table = self.core_iface.core_table
        connection = self.engine.connect()
        transaction = connection.begin()
        query = core_table.select(
            whereclause=(core_table.c.id == object_id))
        result = connection.execute(query).fetchone()
        transaction.commit()
        connection.close()
        if not result:
            err = 'No lama object with id {} in database table {}'.format(
                object_id, core_table.name)
            raise rospy.ServiceException(err)

        # Ensure that the descriptor exists in the database.
        table_name = msg.interface_name
        if not self.core_iface.has_table(table_name):
            err = 'No interface {} in the database'.format(
                msg.interface_name)
            raise rospy.ServiceException(err)
        table = self.core_iface.metadata.tables[table_name]
        desc_id = msg.descriptor_id
        connection = self.engine.connect()
        transaction = connection.begin()
        query = table.select(
            whereclause=(table.c.id == desc_id))
        result = connection.execute(query).fetchone()
        transaction.commit()
        connection.close()
        if not result:
            err = 'No descriptor with id {} in database table {}'.format(
                desc_id, table.name)
            raise rospy.ServiceException(err)

        # Add the descriptor to the descriptor table.
        time = rospy.Time.now()
        connection = self.engine.connect()
        transaction = connection.begin()
        insert_args = {
            'object_id': object_id,
            'descriptor_id': desc_id,
            'interface_name': table_name,
            'timestamp_secs': time.secs,
            'timestamp_nsecs': time.nsecs,
        }
        connection.execute(self.core_iface.descriptor_table.insert(),
                           insert_args)
        transaction.commit()
        connection.close()

        response = ActOnMapResponse()
        return response

    def get_lama_object_list(self, msg, object_type):
        """Retrieve all elements of a given type from the database

        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        - object_type: LamaObject.VERTEX or LamaObject.EDGE.
        """
        # Make the transaction for the core table.
        table = self.core_iface.core_table
        connection = self.engine.connect()
        transaction = connection.begin()
        query = table.select(
            whereclause=(table.c.type == object_type))
        results = connection.execute(query).fetchall()
        transaction.commit()
        connection.close()

        response = ActOnMapResponse()
        if results:
            for result in results:
                lama_object = self.core_iface.get_lama_object(result['id'])
                response.objects.append(lama_object)
        return response

    def get_vertex_list(self, msg):
        """Retrieve all vertices from the database

        Callback for GET_VERTEX_LIST.
        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        return self.get_lama_object_list(msg, LamaObject.VERTEX)

    def get_edge_list(self, msg):
        """Retrieve all edges from the database

        Callback for GET_EDGE_LIST.
        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        return self.get_lama_object_list(msg, LamaObject.EDGE)

    def get_descriptor_links(self, msg):
        """Retrieve DescriptorLink associated with a LamaObject and an interface

        Callback for GET_DESCRIPTOR_LINKS.
        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        response = ActOnMapResponse()
        response.objects.append(msg.object)
        get_desc_links = self.core_iface.get_descriptor_links
        response.descriptor_links = get_desc_links(msg.object.id,
                                                   msg.interface_name)
        return response

    def get_neighbor_vertices(self, msg):
        """Retrieve all neighbor vertices from the database

        Callback for GET_NEIGHBOR_VERTICES.
        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        # TODO: Discuss with Karel what this action does.
        rospy.logerr('GET_NEIGHBOR_VERTICES not implemented')
        response = ActOnMapResponse()
        return response

    def get_outgoing_edges(self, msg):
        """Retrieve edges starting at a given vertex

        Callback for GET_OUTGOING_EDGES.
        Return an instance of ActOnMap response containing edges (instances
        of LamaObject) starting at the given vertex.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        coretable = self.core_iface.core_table
        reftable = self.core_iface.core_obj_ref_table
        query = select([coretable.c.id], from_obj=[coretable, reftable])
        query = query.where(reftable.c['_value_'] == msg.object.id)
        query = query.where(reftable.c['seq_num'] == 0)
        query = query.where(coretable.c['id'] == reftable.c['parent_id'])

        connection = self.engine.connect()
        transaction = connection.begin()
        results = connection.execute(query).fetchall()
        transaction.commit()
        connection.close()
        if not results:
            err = 'No lama object with id {} in database table {}'.format(
                msg.object.id, coretable.name)
            raise rospy.ServiceException(err)
        response = ActOnMapResponse()
        for result in results:
            edge = self.core_iface.get_lama_object(result['id'])
            response.objects.append(edge)
        return response


def core_interface():
    """Return an interface and a map agent classes and run associated services

    Generate an interface class and run the getter, setter, and action services.
    Service definition must be in the form
    GetLamaObject.srv:
      int32 id
      ---
      LamaObject object

    and
    SetLamaObject.srv:
      LamaObject object
      ---
      int32 id

    This function should be called only once with each parameter set because
    it starts ROS services and an error is raised if services are started
    twice.
    """
    iface = CoreDBInterface(g_engine_name, start=True)
    map_agent_iface = MapAgentInterface(start=True)
    return iface, map_agent_iface
