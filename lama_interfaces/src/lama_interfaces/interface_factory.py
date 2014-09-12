from StringIO import StringIO

import rospy

import sqlalchemy
from sqlalchemy.types import Integer, Binary, String

from db_interface import DBInterfaceAbstract

# create engine
# TODO according the rosparam server
_engine = sqlalchemy.create_engine('sqlite:///created.sql')


class DBInterface(DBInterfaceAbstract):
    @property
    def interface_type(self):
        return "serialization"

    def getter(self, msg):
        """Execute the getter service and return the response"""
        # Create an instance of response.
        response = self.getter_service_class._response_class()

        # Make the transaction.
        connection = self.engine.connect()
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
        connection = self.engine.connect()
        transaction = connection.begin()
        insert_args = {'serialized_content': buf.getvalue()}
        result = connection.execute(self.table.insert(), insert_args)
        return_id = result.inserted_primary_key[0]
        transaction.commit()
        connection.close()

        # Return a setter response instance with the descriptor identifier.
        response = self.setter_service_class._response_class()
        response.id.descriptor_id = return_id
        response.id.interface_name = self.interface_type
        return response

    def _generateSchema(self):
        """Generate schema from response class

        Recusrively generate tables for the type of the 'descriptor' variable
        in the response class.
        """
        # Add the type description and check for conflicting interface+type.
        self._addInterfaceDescription()

        column_id = sqlalchemy.Column('id', Integer,
                                      primary_key=True,
                                      nullable=False)
        column_content = sqlalchemy.Column('serialized_content', Binary)
        self.table = sqlalchemy.Table(self.interface_name,
                                      self.metadata,
                                      column_id,
                                      column_content,
                                      extend_existing=True)

        # Create the tables in database.
        self.metadata.create_all()


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
    if getter_srv_msg.endswith('.srv'):
        getter_srv_msg = getter_srv_msg[:-4]
    if setter_srv_msg.endswith('.srv'):
        setter_srv_msg = setter_srv_msg[:-4]
    iface = DBInterface(_engine, interface_name,
                        getter_srv_msg, setter_srv_msg)
    return iface
