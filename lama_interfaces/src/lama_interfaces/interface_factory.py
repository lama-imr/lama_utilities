from StringIO import StringIO

import rospy

import sqlalchemy
from sqlalchemy.types import Integer, Binary

from abstract_db_interface import AbstractDBInterface

# sqlalchemy engine (argument to sqlalchemy.create_engine)
g_engine_name = rospy.get_param('/database_engine', 'sqlite:///created.sqlite')


class DBInterface(AbstractDBInterface):
    @property
    def interface_type(self):
        return "serialization"

    def getter_callback(self, msg):
        """Execute the getter service and return the response"""
        # Create an instance of response.
        response = self.getter_service_class._response_class()

        # Make the transaction.
        id_ = msg.id
        query = self.table.select(whereclause=(self.table.c.id == id_))
        connection = self.engine.connect()
        transaction = connection.begin()
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

    def setter_callback(self, msg):
        """Execute the setter service and return the reponse"""
        buf = StringIO()
        msg.serialize(buf)

        # Make the transaction.
        insert_args = {'serialized_content': buf.getvalue()}
        connection = self.engine.connect()
        transaction = connection.begin()
        result = connection.execute(self.table.insert(), insert_args)
        transaction.commit()
        connection.close()
        return_id = result.inserted_primary_key[0]

        # Return a setter response instance with the descriptor identifier.
        response = self.setter_service_class._response_class()
        response.id = return_id
        self._set_timestamp(rospy.Time.now())
        return response

    def _generate_schema(self):
        """Generate schema from response class

        Recusrively generate tables for the type of the 'descriptor' variable
        in the response class.
        """
        # Add the type description and check for conflicting interface+type.
        self._add_interface_description()

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
            int32 id
            ---
            * descriptor
    - setter_srv_msg: str, identifies the service message used when adding
      a descriptor to the database. For example
      'lama_interfaces/SetVectorLaserScan'.
      Service definition must be in form:
            * descriptor
            ---
            int32 id

    This function should be called only once with each parameter set because
    it starts ROS services and an error is raised if services are started
    twice.
    """
    if getter_srv_msg.endswith('.srv'):
        getter_srv_msg = getter_srv_msg[:-4]
    if setter_srv_msg.endswith('.srv'):
        setter_srv_msg = setter_srv_msg[:-4]
    iface = DBInterface(g_engine_name, interface_name,
                        getter_srv_msg, setter_srv_msg,
                        start=True)
    return iface
