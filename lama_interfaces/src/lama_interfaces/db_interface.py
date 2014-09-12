# Base class for database interface.

from abc import ABCMeta, abstractmethod, abstractproperty
import sqlalchemy

import rospy
import roslib

# Table name for type description
_interfaces_table_name = 'map_interfaces'


class DBInterfaceAbstract(object):
    __metaclass__ = ABCMeta
    def __init__(self, engine, interface_name, getter_srv_msg, setter_srv_msg):
        if '@' in interface_name:
            rospy.logerr('@ not allowd in interface name')
            raise ValueError('@ not allowd in interface name')

        get_srv_class = roslib.message.get_service_class(getter_srv_msg)
        set_srv_class = roslib.message.get_service_class(setter_srv_msg)

        rospy.logdebug('Interface: {} ({},{})'.format(interface_name,
                                                      getter_srv_msg,
                                                      setter_srv_msg))
        rospy.logdebug('Getter class {}'.format(get_srv_class))
        rospy.logdebug('Getter request slots: {}'.format(
            get_srv_class._request_class.__slots__))
        rospy.logdebug('Getter response slots: {}'.format(
            get_srv_class._response_class.__slots__))
        rospy.logdebug('Setter class {}'.format(set_srv_class))
        rospy.logdebug('Setter request slots: {}'.format(
            set_srv_class._request_class.__slots__))
        rospy.logdebug('Setter response slots: {}'.format(
            set_srv_class._response_class.__slots__))

        # getter class
        self.getter_service_name = interface_name + '_getter'
        self.getter_service_class = get_srv_class
        # setter class
        self.setter_service_name = interface_name + '_setter'
        self.setter_service_class = set_srv_class
        # interface name
        self.interface_name = interface_name

        # Database related attributes.
        self.engine = engine
        self.metadata = sqlalchemy.MetaData()
        self.metadata.bind = self.engine
        # Read tables from the possibly existing database.
        self.metadata.reflect()
        # Add the table for type description.
        self.interface_table = self._interfaceTable()
        self.metadata.create_all()
        # Create new tables.
        self._generateSchema()

        # Start the services.
        self.getter_service_proxy = rospy.Service(self.getter_service_name,
                                                  self.getter_service_class,
                                                  self.getter)
        self.setter_service_proxy = rospy.Service(self.setter_service_name,
                                                  self.setter_service_class,
                                                  self.setter)
        rospy.loginfo('Services %s and %s started',
                      self.getter_service_name, self.setter_service_name)

    @abstractproperty
    def interface_type(self):
        pass

    @abstractmethod
    def _generateSchema():
        pass

    @abstractmethod
    def getter(self):
        pass

    @abstractmethod
    def setter(self):
        pass

    def _interfaceTable(self):
        """Return the table for the type description (may already exists).

        Return a table with columns ('interface_name', 'message_type',
        interface_type, timestamp_secs, timestamp_nsecs). If the
        table already exists, it will be returned.
        """
        table = sqlalchemy.Table(_interfaces_table_name,
                                 self.metadata,
                                 sqlalchemy.Column('interface_name',
                                                   sqlalchemy.String,
                                                   unique=True),
                                 sqlalchemy.Column('message_type',
                                                   sqlalchemy.String),
                                 sqlalchemy.Column('interface_type',
                                                   sqlalchemy.String),
                                 sqlalchemy.Column('timestamp_secs',
                                                   sqlalchemy.BigInteger),
                                 sqlalchemy.Column('timestamp_nsecs',
                                                   sqlalchemy.BigInteger),
                                 extend_existing=True)

        return table

    def _addInterfaceDescription(self):
        """Add the inteface description if not already existing

        Add the inteface description with unconflicting
        (interface_name / message_type / interface_type).

        A ValueError is raised if a table with the same name (i.e. same
        interface_name) but different message type and interface type already
        exists.
        """
        # Check for an existing table.
        table = self.interface_table
        name = self.interface_name
        msg_type = self.getter_service_class._response_class._slot_types[0]

        connection = self.engine.connect()
        transaction = connection.begin()
        query = table.select(whereclause=(table.c.interface_name == name))
        result = connection.execute(query).fetchone()
        transaction.commit()
        connection.close()

        table_exists = False
        if result:
            table_exists = True
            if (result['message_type'] != msg_type or
                result['interface_type'] != self.interface_type):
                err = ('A table "{}" with message type "{}" and interface ' +
                       'type "{}" already exists, cannot change to ' +
                       '"{}"/"{}"'.format(
                           name,
                           result['message_type'], result['interface_type'],
                           msg_type), self.interface_type)
                rospy.logfatal(err)
                raise ValueError(err)

        # Add the table description if necessary.
        if not table_exists:
            connection = self.engine.connect()
            transaction = connection.begin()
            insert_args = {
                'interface_name': name,
                'message_type': msg_type,
                'interface_type': self.interface_type,
            }
            connection.execute(table.insert(), insert_args)
            transaction.commit()
            connection.close()
