#!/usr/bin/env python

from sqlalchemy import create_engine
from sqlalchemy import select
from sqlalchemy import MetaData
from sqlalchemy import ForeignKey
from sqlalchemy import Table
from sqlalchemy import Column
from sqlalchemy import types

import rospy
import roslib.msgs
import roslib.message

from lama_interfaces.msg import MapAction
from lama_interfaces.msg import LamaObject
from lama_interfaces.msg import LamaDescriptorIdentifier

g_default_core_table_name = 'lama_objects'
g_default_descriptor_table_name = 'lama_descriptor_ids'

# create engine
# TODO according  the rosparam server
_engine = create_engine('sqlite:///created.sql')


class CoreDBInterface(object):
    def __init__(self, interface_name=None, descriptor_table_name=None):
        if not interface_name:
            interface_name = g_default_core_table_name
        if not descriptor_table_name:
            descriptor_table_name = g_default_descriptor_table_name
        get_srv_type = 'lama_interfaces/GetLamaObject'
        set_srv_type = 'lama_interfaces/SetLamaObject'
        action_srv_type = 'lama_interfaces/ActOnMap'
        get_srv_class = roslib.message.get_service_class(get_srv_type)
        set_srv_class = roslib.message.get_service_class(set_srv_type)
        srv_action_class = roslib.message.get_service_class(action_srv_type)

        # getter class.
        self.getter_service_name = 'lama_object_getter'
        self.getter_service_class = get_srv_class
        # setter class.
        self.setter_service_name = 'lama_object_setter'
        self.setter_service_class = set_srv_class
        # Action class.
        self.action_service_name = 'lama_map_agent'
        self.action_service_class = srv_action_class

        self.interface_name = interface_name
        self.descriptor_table_name = descriptor_table_name

        # Database related attributes.
        self.engine = _engine
        self.metadata = MetaData()
        self.metadata.bind = self.engine
        # Read tables from the possibly existing database.
        self.metadata.reflect()
        self._generate_schema()

    def _generate_schema(self):
        """Create the SQL tables"""
        self._generate_core_table()
        self._generate_descriptor_table()
        self.metadata.create_all()

    def _generate_core_table(self):
        """Create the SQL tables for LamaObject messages"""
        # The table format is hard-coded.
        table = Table(self.interface_name,
                      self.metadata,
                      Column('id', types.Integer, primary_key=True))
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
        """Create the SQL tables for descritptors"""
        table = Table(self.descriptor_table_name,
                      self.metadata,
                      Column('id', types.Integer, primary_key=True))
        table.append_column(Column('object_id',
                                   types.Integer,
                                   ForeignKey(self.interface_name + '.id')))
        table.append_column(Column('descriptor_id', types.Integer,
                                   unique=True))
        table.append_column(Column('interface_name', types.String))
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
        lama_object = self._get_lama_object(id_)
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
        response.id = self._set_lama_object(msg.object)
        return response

    def action_callback(self, msg):
        """Callback of ActOnMap service

        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        action = msg.action.action
        if action == MapAction.PUSH_VERTEX:
            r = self.push_lama_object(msg)
        elif action == MapAction.PULL_VERTEX:
            r = self.pull_lama_object(msg)
        elif action == MapAction.ASSIGN_DESCRIPTOR_VERTEX:
            r = self.assign_descriptor_to_lama_object(msg)
        elif action == MapAction.PUSH_EDGE:
            r = self.push_lama_object(msg)
        elif action == MapAction.PULL_EDGE:
            r = self.pull_lama_object(msg)
        elif action == MapAction.ASSIGN_DESCRIPTOR_EDGE:
            r = self.assign_descriptor_to_lama_object(msg)
        elif action == MapAction.GET_VERTEX_LIST:
            r = self.get_vertex_list(msg)
        elif action == MapAction.GET_EDGE_LIST:
            r = self.get_edge_list(msg)
        elif action == MapAction.GET_NEIGHBOR_VERTICES:
            r = self.get_neighbor_vertices(msg)
        elif action == MapAction.GET_OUTGOING_EDGES:
            r = self.get_outgoing_edges(msg)
        else:
            # TODO: find a mechanism for non-implemented actions.
            rospy.logerr('Action {} not implemented'.format(msg.action))
            r = self.action_service_class._response_class()
        return r

    def _get_lama_object(self, id_):
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
            rospy.logerr(err)
            raise ValueError(err)

        lama_object = LamaObject()
        lama_object.id = id_
        lama_object.id_in_world = result['id_in_world']
        lama_object.name = result['name']
        lama_object.type = result['type']

        # Make the transaction for the array 'references'.
        if lama_object.type == LamaObject.EDGE:
            query = self.core_obj_ref_table.select(
                whereclause=(self.core_obj_ref_table.parent_id == id_))
            connection = self.engine.connect()
            transaction = connection.begin()
            refs = connection.execute(query).fetchall()
            transaction.commit()
            connection.close()
            lama_object.references = [0] * 2
            for ref in refs:
                index = ref['seq_num']
                val = ref['_value_']
                lama_object.references[index] = val

        return lama_object

    def _get_lama_objects(self, id_in_world):
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
            rospy.logerr(err)
            raise ValueError(err)

        lama_objects = []
        for result in results:
            lama_object = LamaObject()
            lama_object.id = result['id']
            lama_object.id_in_world = id_in_world
            lama_object.name = result['name']
            lama_object.type = result['type']

            # Make the transaction for the array 'references'.
            if lama_object.type == LamaObject.EDGE:
                connection = self.engine.connect()
                transaction = connection.begin()
                query = self.core_obj_ref_table.select(
                    whereclause=(self.core_obj_ref_table.parent_id == result['id']))
                refs = connection.execute(query).fetchall()
                transaction.commit()
                connection.close()
                lama_object.references = [0] * 2
                for ref in refs:
                    index = ref['seq_num']
                    val = ref['_value_']
                    lama_object.references[index] = val
            lama_objects.append(lama_object)

        return lama_objects

    def _get_lama_object_descriptor_ids(self, id_):
        """Retrieve descriptor identifiers associated with a Lama object

        Return a list of LamaDescriptorIdentifier.

        Parameters
        ----------
        - id_: int, lama object id in the database.
        """
        desc_ids = []
        # Make the transaction from the descriptor table.
        connection = self.engine.connect()
        transaction = connection.begin()
        query = self.descriptor_table.select(
            whereclause=(self.core_table.c.object_id == id_))
        results = connection.execute(query).fetchall()
        transaction.commit()
        connection.close()
        if not results:
            err = 'No lama object with id {} in database table {}'.format(
                id_, self.descriptor_table.name)
            rospy.logerr(err)
            raise ValueError(err)
        for result in results:
            desc_id = LamaDescriptorIdentifier()
            desc_id.object_id = id_
            desc_id.deserialize = result['descriptor_id']
            desc_id.interface_name = result['interface_name']
            desc_ids.append(desc_id)
        return desc_ids

    def _set_lama_object(self, lama_object):
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

        return object_id

    def push_lama_object(self, msg):
        """Add a LaMa object to the database

        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        response = self.action_service_class._response_class()
        response.id = self._set_lama_object(msg.object)
        return response

    def pull_lama_object(self, msg):
        """Retrieve a LaMa object from the database

        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        response = self.action_service_class._response_class()
        id_ = msg.object.id
        response.objects.append(self._get_lama_object(id_))
        response.descriptors = self._get_lama_object_descriptor_ids(id_)
        return response

    def assign_descriptor_to_lama_object(self, msg):
        """Add a descriptor to a vertex

        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request. All information must be in
            msg.descriptor (msg.object is ignored).
        """
        # Ensure that the lama object exists in the core table.
        object_id = msg.desciptor.object_id
        connection = self.engine.connect()
        transaction = connection.begin()
        query = self.core_table.select(
            whereclause=(self.core_table.c.id == object_id))
        result = connection.execute(query).fetchone()
        transaction.commit()
        connection.close()
        if not result:
            err = 'No lama object with id {} in database table {}'.format(
                object_id, self.core_table.name)
            rospy.logerr(err)
            raise ValueError(err)

        # Ensure that the descriptor exists in the database.
        table_name = msg.desciptor.interface_name
        if table_name not in self.metadata.tables:
            err = 'No interface {} in the database'.format(
                msg.desciptor.interface_name)
            rospy.logerr(err)
            raise ValueError(err)
        table = self.metadata.tables[table_name]
        desc_id = msg.desciptor.descriptor_id
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
            rospy.logerr(err)
            raise ValueError(err)

        # Add the descriptor to the descriptor table.
        connection = self.engine.connect()
        transaction = connection.begin()
        insert_args = {
            'object_id': object_id,
            'descriptor_id': desc_id,
            'interface_name': table_name,
        }
        connection.execute(self.descriptor_table.insert(), insert_args)
        transaction.commit()
        connection.close()

        response = self.action_service_class._response_class()
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
        connection = self.engine.connect()
        transaction = connection.begin()
        query = self.core_table.select(
            whereclause=(self.core_table.c.type == object_type))
        results = connection.execute(query).fetchall()
        transaction.commit()
        connection.close()

        response = self.action_service_class._response_class()
        if results:
            for result in results:
                lama_object = self._get_lama_objects(result['id'])
                response.objects.append(lama_object)
        return response

    def get_vertex_list(self, msg):
        """Retrieve all vertices from the database

        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        return self.get_lama_object_list(msg, LamaObject.VERTEX)

    def get_edge_list(self, msg):
        """Retrieve all edges from the database

        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        return self.get_lama_object_list(msg, LamaObject.EDGE)

    def get_neighbor_vertices(self, msg):
        """Retrieve all neighbor vertices from the database

        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        # TODO: Discuss with Karel what this action does.
        rospy.logerr('GET_NEIGHTBOR_VERTICES not implemented')
        response = self.action_service_class._response_class()
        return response

    def get_outgoing_edges(self, msg):
        """Return a core reponse message with edges starting at a vertex

        Return an instance of ActOnMap response containing edges (instances
        of LamaObject) starting at the given vertex.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        coretable = self.core_table
        reftable = self.core_obj_ref_table
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
                msg.object.id, self.core_table.name)
            rospy.logerr(err)
            raise ValueError(err)
        response = self.action_service_class._response_class()
        for result in results:
            edge = self._get_lama_objects(result['id'])
            response.objects.append(edge)
        return response


def core_interface():
    """Return an interface class and run its associated services

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
    """
    iface = CoreDBInterface()
    rospy.Service(iface.getter_service_name,
                  iface.getter_service_class,
                  iface.getter_callback)
    rospy.Service(iface.setter_service_name,
                  iface.setter_service_class,
                  iface.setter_callback)
    rospy.Service(iface.action_service_name,
                  iface.action_service_class,
                  iface.action_callback)
    return iface
