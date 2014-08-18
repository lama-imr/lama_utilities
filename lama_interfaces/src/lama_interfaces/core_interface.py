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

from lama_interfaces.msg import LamaMapAction
from lama_interfaces.msg import LamaObjectIdentifier

g_default_core_table_name = 'core'
g_default_descriptor_table_name = 'lama_descriptors'

# create engine
# TODO according  the rosparam server
_engine = create_engine('sqlite:///created.sql')


class CoreDBInterface(object):
    def __init__(self, service_name=None, descriptor_table_name=None):
        if not service_name:
            service_name = g_default_core_table_name
        if not descriptor_table_name:
            descriptor_table_name = g_default_descriptor_table_name
        srv_type = 'lama_interfaces/lmi_core'
        srv_get_class = roslib.message.get_service_class(srv_type + '_get')
        srv_set_class = roslib.message.get_service_class(srv_type + '_set')
        srv_action_class = roslib.message.get_service_class(srv_type)

        self._getter_class = srv_get_class
        self._setter_class = srv_set_class
        self._action_class = srv_action_class
        self.service_name = service_name
        self.descriptor_table_name = descriptor_table_name
        self.metadata = MetaData()
        self._generate_schema()

    def _generate_schema(self):
        """Create the SQL tables"""
        self._generate_core_table()
        self._generate_descriptor_table()
        self.metadata.create_all(_engine)

    def _generate_core_table(self):
        """Create the SQL tables for LamaObjectIdentifier messages"""
        # The table format is hard-coded.
        table = Table(self.service_name,
                      self.metadata,
                      Column('id', types.Integer, primary_key=True))
        table.append_column(Column('object_id',
                                   types.Integer,
                                   ForeignKey(self.service_name + '.id'),
                                   unique=True))
        table.append_column(Column('object_name', types.String))
        table.append_column(Column('object_type', types.String))
        self.core_table = table

        table = Table(self.service_name + '@object_references@_data_',
                      self.metadata)
        table.append_column(Column('id', types.Integer, primary_key=True))
        table.append_column(Column('seq_num', types.Integer))
        table.append_column(Column('parent_id', types.Integer,
                                   ForeignKey(self.service_name + '.id')))
        table.append_column(Column('_value_', types.Integer))
        self.core_obj_ref_table = table

    def _generate_descriptor_table(self):
        """Create the SQL tables for descritptors"""
        table = Table(self.descriptor_table_name,
                      self.metadata,
                      Column('id', types.Integer, primary_key=True))
        table.append_column(Column('object_id',
                                   types.Integer,
                                   ForeignKey(self.service_name + '.id')))
        table.append_column(Column('descriptor_id', types.Integer,
                                   unique=True))
        table.append_column(Column('interface_name', types.String))
        self.descriptor_table = table

    def getter(self, msg):
        """Get a LamaObjectIdentifier from the database

        Get a LamaObjectIdentifier from the database, from its
        database-specific id.

        Parameters
        ----------
        - msg: an instance of lmi_core_get.srv request.
        """
        # Create an instance of getter response.
        response = self._getter_class._response_class()

        # Make the transaction for the core table.
        connection = _engine.connect()
        id_ = msg.id
        query = self.core_table.select(
            whereclause=(self.core_table.c.id == id_))
        result = connection.execute(query).fetchone()
        connection.close()
        if not result:
            err = 'No element with id {} in database table {}'.format(
                id_, self.core_table.name)
            rospy.logerr(err)
            raise ValueError(err)
        response.object_id = result['object_id']
        response.object_name = result['object_name']
        response.object_type = result['object_type']

        # Make the transaction for the array 'object_references'.
        if response.object_type == 'edge':
            connection = _engine.connect()
            query = self.core_obj_ref_table.select(
                whereclause=(self.core_obj_ref_table.parent_id == id_))
            results = connection.execute(query).fetchall()
            connection.close()
            response.object_references = [0] * 2
            for result in results:
                index = result['seq_num']
                val = result['_value_']
                response.object_references[index] = val

        return response

    def setter(self, msg):
        """Add a LamaObjectIdentifier message to the database

        Parameters
        ----------
        - msg: an instance of lmi_core_set.srv request.
        """
        # Create an instance of setter response.
        response = self._setter_class._response_class()

        # Make the transaction for the core table.
        connection = _engine.connect()
        transaction = connection.begin()
        insert_args = {
            'object_id': msg.object_id,
            'object_name': msg.object_name,
            'object_type': msg.object_type,
        }
        result = connection.execute(self.core_table.insert(), insert_args)
        return_id = result.inserted_primary_key[0]
        transaction.commit()
        connection.close()

        # Make the transaction for the 'object_references' array.
        if msg.object_type == 'edge':
            connection = _engine.connect()
            transaction = connection.begin()
            for i, v in enumerate(msg.object_references):
                insert_args = {
                    'seq_num': i,
                    'parent_id': return_id,
                    '_value_': v,
                }
                connection.execute(self.core_table.insert(), insert_args)
            transaction.commit()
            connection.close()

        response.id.descriptor_id = return_id
        response.id.interface_name = self.service_name
        return response

    def action_callback(self, msg):
        action = msg.action.action
        if action == LamaMapAction.PUSH_VERTEX:
            r = self.push_lama_object(msg)
        elif action == LamaMapAction.PULL_VERTEX:
            r = self.pull_lama_object(msg)
        elif action == LamaMapAction.ASSIGN_DESCRIPTOR_VERTEX:
            r = self.assign_descriptor_to_lama_object(msg)
        elif action == LamaMapAction.PUSH_EDGE:
            r = self.push_lama_object(msg)
        elif action == LamaMapAction.PULL_EDGE:
            r = self.pull_lama_object(msg)
        elif action == LamaMapAction.ASSIGN_DESCRIPTOR_EDGE:
            r = self.assign_descriptor_to_lama_object(msg)
        elif action == LamaMapAction.GET_VERTEX_LIST:
            r = self.get_vertex_list(msg)
        elif action == LamaMapAction.GET_EDGE_LIST:
            r = self.get_edge_list(msg)
        elif action == LamaMapAction.GET_NEIGHBOR_VERTICES:
            r = self.get_neighbor_vertices(msg)
        elif action == LamaMapAction.GET_OUTGOING_EDGES:
            r = self.get_outgoing_edges(msg)
        else:
            rospy.logerr('Action {} not implemented'.format(msg.action))
            r = None
        return r

    def _get_lama_object(self, object_id):
        """Get a vertex or an edge from its object_id

        Parameters
        ----------
        - object_id: int.
        """
        lama_object = LamaObjectIdentifier()
        lama_object.object_id = object_id

        # Make the transaction for the core table.
        connection = _engine.connect()
        query = self.core_table.select(
            whereclause=(self.core_table.c.object_id == object_id))
        result = connection.execute(query).fetchone()
        connection.close()
        if not result:
            err = 'No element with object_id {} in database table {}'.format(
                object_id, self.core_table.name)
            rospy.logerr(err)
            raise ValueError(err)
        lama_object.object_name = result['object_name']
        lama_object.object_type = result['object_type']

        # Make the transaction for the array 'object_references'.
        if lama_object.object_type == 'edge':
            connection = _engine.connect()
            query = self.core_obj_ref_table.select(
                whereclause=(self.core_obj_ref_table.parent_id == result['id']))
            results = connection.execute(query).fetchall()
            lama_object.object_references = [0] * 2
            for result in results:
                index = result['seq_num']
                val = result['_value_']
                lama_object.object_references[index] = val
            connection.close()

        return lama_object

    def push_lama_object(self, msg):
        """Add a LaMa object to the database)

        Parameters
        ----------
        - msg: an instance of lmi_core.srv request.
        """
        request = self._setter_class._request_class()
        request.object = msg.object
        self.setter(request)
        return

    def pull_lama_object(self, msg):
        """Retrieve a LaMa object from the database

        Parameters
        ----------
        - msg: an instance of lmi_core.srv request.
        """
        return self._get_lama_object(msg.object.object_id)

    def assign_descriptor_to_lama_object(self, msg):
        """Add a descriptor to a vertex

        Parameters
        ----------
        - msg: an instance of lmi_core.srv request.
        """
        # Make the query from the core table.
        connection = _engine.connect()
        query = self.core_table.select(
            whereclause=(self.core_table.c.object_id == msg.object.object_id))
        result = connection.execute(query).fetchone()
        connection.close()
        if not result:
            err = 'No element with object_id {} in database table {}'.format(
                msg.object.object_id, self.core_table.name)
            rospy.logerr(err)
            raise ValueError(err)
        id_ = result['id']

        # Add the descriptor to the descriptor table.
        connection = _engine.connect()
        transaction = connection.begin()
        insert_args = {
            'object_id': id_,
            'descriptor_id': msg.desciptor.descriptor_id,
            'interface_name': msg.descriptor.interface_name,
        }
        connection.execute(self.descriptor_table.insert(), insert_args)
        transaction.commit()
        connection.close()

    def get_lama_object_list(self, msg, object_type):
        """Retrieve all elements of a given type from the database

        Parameters
        ----------
        - msg: an instance of lmi_core.srv request.
        - object_type: 'vertex' or 'edge'.
        """
        # Make the transaction for the core table.
        connection = _engine.connect()
        query = self.core_table.select(
            whereclause=(self.core_table.c.object_type == object_type))
        results = connection.execute(query).fetchall()
        connection.close()

        response = self._action_class._response_class()
        if results:
            for result in results:
                lama_object = self._get_lama_object(result['object_id'])
                response.objects.append(lama_object)
        return response

    def get_vertex_list(self, msg):
        """Retrieve all vertices from the database

        Parameters
        ----------
        - msg: an instance of lmi_core.srv request.
        """
        self.get_lama_object_list(msg, 'vertex')

    def get_edge_list(self, msg):
        """Retrieve all edges from the database

        Parameters
        ----------
        - msg: an instance of lmi_core.srv request.
        """
        self.get_lama_object_list(msg, 'edge')

    def get_neighbor_vertices(self, msg):
        rospy.logerr('GET_NEIGHTBOR_VERTICES not implemented')
        return

    def get_outgoing_edges(self, msg):
        """Return a core reponse message with edges starting at a vertex

        Return an instance or lmi_core.srv response containing edges (instances
        of LamaObjectIdentifier) starting
        at the given vertex.

        Parameters
        ----------
        - msg: an instance of lmi_core.srv request.
        """
        coretable = self.core_table
        reftable = self.core_obj_ref_table
        query = select([coretable.c.id], from_obj=[coretable, reftable])
        query = query.where(reftable.c['_value_'] == msg.object.object_id)
        query = query.where(reftable.c['seq_num'] == 0)
        query = query.where(coretable.c['id'] == reftable.c['parent_id'])

        connection = _engine.connect()
        results = connection.execute(query).fetchall()
        connection.close()
        response = self._action_class._response_class()
        if not results:
            err = 'No element with object_id {} in database table {}'.format(
                msg.object.object_id, self.core_table.name)
            rospy.logerr(err)
            raise ValueError(err)
        for result in results:
            edge = self._get_lama_object(result['object_id'])
            response.objects.append(edge)
        return response


def core_interface():
    """Return an interface class and run its associated services

    Generate an interface class and run the getter, setter, and action services.
    Service definition must be in the form *_get and *_set where
    *_get
      lama_object_identifier id
      ---
      * object

    and
    *_set
      * object
      ---
      lama_object_identifier id
    """
    service_name = g_default_core_table_name
    iface = CoreDBInterface()
    sub_name = 'lmi_' + service_name
    # print "starting interface", sub_name
    rospy.Service(sub_name + '_getter', iface._getter_class, iface.getter)
    rospy.Service(sub_name + '_setter', iface._setter_class, iface.setter)
    rospy.Service(sub_name + '_action', iface._action_class,
                  iface.action_callback)
    return iface
