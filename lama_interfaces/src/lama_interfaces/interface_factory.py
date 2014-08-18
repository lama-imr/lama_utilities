#!/usr/bin/env python

from StringIO import StringIO

import rospy
import roslib.message

import sqlalchemy
from sqlalchemy.types import Integer, Binary, String

_types_table_name = 'message_types'

# create engine
# TODO according the rosparam server
_engine = sqlalchemy.create_engine('sqlite:///created.sql')

_used_srv_names = []


class DBInterface(object):
    def __init__(self, srv_name, g_cls, s_cls):
        # getter class
        self._getter_class = g_cls
        # setter class
        self._setter_class = s_cls
        # service name
        self.srv_name = srv_name
        # sqlalchemy metadata holds the table definition
        self.metadata = sqlalchemy.MetaData(_engine)
        self._generateSchema()

    def getter(self, msg):
        """Execute the getter service and return the response"""
        # Create an instance of response.
        response = self._getter_class._response_class()

        # Make the transaction.
        connection = _engine.connect()
        transaction = connection.begin()
        id_ = msg.id.descriptor_id
        query = self.table.select(whereclause=(self.table.c.id == id_))
        result = connection.execute(query).fetchone()
        if not result:
            err = 'No element with id {} in database table {}'.format(
                id_, self.table.name)
            rospy.logerr(err)
            raise ValueError(err)
        transaction.commit()
        connection.close()

        response.deserialize(result['serialized_content'])
        return response

    def setter(self, msg):
        """Execute the setter service and return the reponse"""
        buf = StringIO()
        msg.serialize(buf)

        # Make the transaction.
        connection = _engine.connect()
        transaction = connection.begin()
        insert_args = {'serialized_content': buf.getvalue()}
        result = connection.execute(self.table.insert(), insert_args)
        return_id = result.inserted_primary_key[0]
        transaction.commit()
        connection.close()

        # Return a setter response instance with the descriptor identifier.
        response = self._setter_class._response_class()
        response.id.descriptor_id = return_id
        response.id.interface_name = self.srv_name
        return response

    def _generateSchema(self):
        """Generate schema from response class

        Recusrively generate tables for the type of the 'descriptor' variable
        in the response class.
        """
        column_id = sqlalchemy.Column('id', Integer,
                                      primary_key=True,
                                      nullable=False)
        column_content = sqlalchemy.Column('serialized_content', Binary)
        self.table = sqlalchemy.Table(self.srv_name,
                                      self.metadata,
                                      column_id,
                                      column_content)

        # Add a table for the type description (may already exists).
        table = sqlalchemy.Table(_types_table_name,
                                 self.metadata,
                                 sqlalchemy.Column('table_name',
                                                   String,
                                                   unique=True),
                                 sqlalchemy.Column('message_type', String))

        # Create the tables in database.
        self.metadata.create_all()

        # Add the type description.
        connection = _engine.connect()
        transaction = connection.begin()
        insert_args = {
            'table_name': self.srv_name,
            'message_type': self._getter_class._response_class._slot_types[0]
        }
        connection.execute(table.insert(), insert_args)
        transaction.commit()
        connection.close()


def interface_factory(service_name, srv_type):
    """generate the interface class and run the getter and setter services

    service definition must be in form of *_get and *_set where
        *_get:
            LamaDescriptorIdentifier id
            ---
            * descriptor

        and
        *_set
            * descriptor
            ---
            LamaDescriptorIdentifier id
    """
    if '@' in service_name:
        rospy.logerr('@ not allowd in service name')
        raise ValueError('@ not allowd in service name')
    if service_name in _used_srv_names:
        rospy.logfatal('Service "{}" already in use'.format(service_name))
        raise ValueError('Service "{}" already in use'.format(service_name))

    srv_get_class = roslib.message.get_service_class(srv_type + '_get')
    srv_set_class = roslib.message.get_service_class(srv_type + '_set')

    rospy.logdebug('Getter class {}'.format(srv_get_class))
    rospy.logdebug('Getter request slots: {}'.format(srv_get_class._request_class.__slots__))
    rospy.logdebug('Getter response slots: {}'.format(srv_get_class._response_class.__slots__))
    rospy.logdebug('Setter class {}'.format(srv_set_class))
    rospy.logdebug('Setter request slots: {}'.format(srv_set_class._request_class.__slots__))
    rospy.logdebug('Setter response slots: {}'.format(srv_set_class._response_class.__slots__))

    iface = DBInterface(service_name, srv_get_class, srv_set_class)
    sub_name = 'lmi_' + service_name
    rospy.logdebug('starting interface', sub_name)
    rospy.Service(sub_name + '_setter', srv_set_class, iface.setter)
    rospy.Service(sub_name + '_getter', srv_get_class, iface.getter)
    return iface
