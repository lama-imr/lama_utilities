#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import sys
import sqlalchemy
from lama_interfaces.interface_factory import DBInterface as SerInterface
from lama_interfaces.cleartext_interface_factory import DBInterface as CtInterface

_services = {
    'sensor_msgs/LaserScan[]': ('lama_interfaces/GetVectorLaserScan',
                                'lama_interfaces/SetVectorLaserScan'),
    'lama_msgs/Crossing': ('lama_msgs/GetCrossing', 'lama_msgs/SetCrossing'),
    'float64': ('lama_interfaces/GetDouble',
                'lama_interfaces/SetDouble'),
}


def read_map_interfaces(metadata):
    table = metadata.tables['map_interfaces']
    query = table.select()
    connection = metadata.bind.connect()
    transaction = connection.begin()
    results = connection.execute(query).fetchall()
    transaction.commit()
    connection.close()
    if not results:
        return None
    classes = {
        'serialization': SerInterface,
        'cleartext': CtInterface,
    }
    interfaces = {}
    for result in results:
        iface_name = result['interface_name']
        cls = classes[result['interface_type']]
        interfaces[iface_name] = cls(
            metadata.bind,
            iface_name,
            _services[result['message_type']][0],
            _services[result['message_type']][1],
            start=False)
    return interfaces


def getter(id_, interface_name):
    interface = interfaces[interface_name]
    request = interface.getter_service_class._request_class()
    request.id = id_
    response = interface.getter(request)
    return response.descriptor

if __name__ == '__main__':
    if len(sys.argv) > 1:
        engine_name = 'sqlite:///' + sys.argv[1]
        engine = sqlalchemy.create_engine(engine_name)
        metadata = sqlalchemy.MetaData(engine)
        metadata.reflect()
        interfaces = read_map_interfaces(metadata)
