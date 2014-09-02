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

_used_interface_names = []


class DBInterface(object):
    def __init__(self, interface_name, getter_srv_msg, setter_srv_msg):
        get_srv_class = roslib.message.get_service_class(getter_srv_msg)
        set_srv_class = roslib.message.get_service_class(setter_srv_msg)

        rospy.logdebug('Getter class {}'.format(get_srv_class))
        rospy.logdebug('Getter request slots: {}'.format(get_srv_class._request_class.__slots__))
        rospy.logdebug('Getter response slots: {}'.format(get_srv_class._response_class.__slots__))
        rospy.logdebug('Setter class {}'.format(set_srv_class))
        rospy.logdebug('Setter request slots: {}'.format(set_srv_class._request_class.__slots__))
        rospy.logdebug('Setter response slots: {}'.format(set_srv_class._response_class.__slots__))

        # getter class
        self.getter_service_name = interface_name + '_getter'
        self.getter_service_class = get_srv_class
        # setter class
        self.setter_service_name = interface_name + '_setter'
        self.setter_service_class = set_srv_class
        # interface name
        self.interface_name = interface_name
        # sqlalchemy metadata holds the table definition
        self.metadata = sqlalchemy.MetaData(_engine)
        self._generateSchema()

    def getter(self, msg):
        """Execute the getter service and return the response"""
        # Create an instance of response.
        response = self.getter_service_class._response_class()

        # Make the transaction.
        connection = _engine.connect()
        transaction = connection.begin()
        id_ = msg.id.descriptor_id
        query = self.table.select(whereclause=(self.table.c.id == id_))
        result = connection.execute(query).fetchone()
        transaction.commit()
        connection.close()
        if not result:
            err = 'No element with id {} in database table {}'.format(
                id_, self.table.name)
            rospy.logerr(err)
            raise ValueError(err)

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
        response = self.setter_service_class._response_class()
        response.id.descriptor_id = return_id
        response.id.interface_name = self.interface_name
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
        self.table = sqlalchemy.Table(self.interface_name,
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
        self._addTypeDescription(table)

    def _addTypeDescription(self, table):
        """Add the type description if not already existing

        An error is raised if a table with the same name (i.e. same
        interface_name) but a different type already exists.
        """
        # Check for an existing table.
        connection = _engine.connect()
        transaction = connection.begin()
        name = self.interface_name
        msg_type = self.getter_service_class._response_class._slot_types[0]
        query = table.select(whereclause=(table.c.table_name == name))
        result = connection.execute(query).fetchone()
        transaction.commit()
        connection.close()
        table_exists = False
        if result:
            table_exists = True
            if result['message_type'] != msg_type:
                err = ('A table "{}" with type "{}" already exists' +
                       ', cannot change to type "{}"'.format(
                           name, result['message_type'], msg_type))
                rospy.logfatal(err)
                raise ValueError(err)

        # Add the table description if necessary.
        if not table_exists:
            connection = _engine.connect()
            transaction = connection.begin()
            insert_args = {
                'table_name': name,
                'message_type': msg_type,
            }
            connection.execute(table.insert(), insert_args)
            transaction.commit()
            connection.close()


def interface_factory(interface_name, getter_srv_msg, setter_srv_msg):
    """generate the interface class and run the getter and setter services

    Example of call: interface_factory('laser_descriptor',
      'lama_interface/get_laser_descriptor',
      'lama_interface/set_laser_descriptor').

    Parameters
    ----------
    - interface_name: str, uniquely identifies an interface between jockeys
      and the database.
    - getter_srv_msg: str, identifies the service message used when retrieving
      a descriptor from the database. For example
      'lama_interfaces/GetVectorLaserScan'.
      Service definition must be in form:
            lama_interfaces/DescriptorIdentifier id
            ---
            * descriptor
    - setter_srv_msg: str, identifies the service message used when adding
      a descriptor to the database. For example
      'lama_interfaces/SetVectorLaserScan'.
      Service definition must be in form:
            * descriptor
            ---
            lama_interfaces/DescriptorIdentifier id
    """
    if '@' in interface_name:
        rospy.logerr('@ not allowd in interface name')
        raise ValueError('@ not allowd in interface name')
    if interface_name in _used_interface_names:
        rospy.logfatal('Interface "{}" already in use'.format(interface_name))
        raise ValueError('Interface "{}" already in use'.format(interface_name))

    if getter_srv_msg.endswith('.srv'):
        getter_srv_msg = getter_srv_msg[:-4]
    if setter_srv_msg.endswith('.srv'):
        setter_srv_msg = setter_srv_msg[:-4]
    iface = DBInterface(interface_name, getter_srv_msg, setter_srv_msg)

    rospy.Service(iface.getter_service_name,
                  iface.getter_service_class,
                  iface.getter)
    rospy.Service(iface.setter_service_name,
                  iface.setter_service_class,
                  iface.setter)
    rospy.loginfo('Services %s and %s started',
                  iface.getter_service_name, iface.setter_service_name)
    return iface
