#!/usr/bin/env python

from __future__ import print_function

import rospy
import roslib.msgs
import roslib.message
from genpy.rostime import Time

import sqlalchemy
from sqlalchemy import types

from db_interface import DBInterfaceAbstract

# Mapping from python types to sqlalchemy types.
type_map = {
    'bool': types.Boolean(),
    'char': types.SmallInteger(unsigned=True),
    'int8': types.SmallInteger(),
    'uint8': types.SmallInteger(unsigned=True),
    'byte': types.SmallInteger(),
    'int16': types.Integer(),
    'uint16': types.Integer(unsigned=True),
    'int32': types.Integer(),
    'uint32': types.Integer(unsigned=True),
    'int64': types.BigInteger(),
    'uint64': types.BigInteger(unsigned=True),
    'float32': types.Float(precision=32),
    'float64': types.Float(precision=64),
    'string': types.Text(),
}

suffix_for_array_table = '_data_'
suffix_for_builtin = '_value_'

_types_table_name = 'message_types'

# create engine
# TODO according  the rosparam server
_engine = sqlalchemy.create_engine('sqlite:///created.sql')

# TODO: remove all occurences of roslib.msgs
def getListItem(l, i):
    if i >= len(l):
        return None
    return l[i]


def setListItem(l, i, y):
    if i >= len(l):
        l.extend([None] * (i + 1 - len(l)))
    l[i] = y


def getMsgSubtypes(msg):
    """Return a dictionnary mapping slot names to slot type"""
    return dict(zip(msg.__slots__, msg._slot_types))


def getMsgArgValue(msg, arg, index=None):
    """Return the value of msg.arg, where arg can be a nested arg

    Return the value of msg.arg, where arg can be a nested arg. For
    example, with odom = nav_msgs.msg.Odometry() the following are valid:
        getMsgArgValue(msg, 'pose') <==> msg.pose
        getMsgArgValue(msg, 'pose.pose.position.x') <==>msg.pose.pose.position.x
    The separator in arg can be either '.' or '@', so that
        getMsgArgValue(msg, 'pose@pose@position@x') <==>msg.pose.pose.position.x
    If arg is the empty string, returns the whole message (even if it's a list).
    If index is not None, the corresponding item list is returned instead of
    the whole list.

    Parameters
    ----------
    - msg: instance of ROS message.
    - arg: str, argument to get from msg. Valid values for arg are given by
        roslib.message.get_printable_message_args(msg).split().
    - index: if not None, msg.arg[index] is returned rather than msg.arg.
        Defaults to None.
    """
    if ('@' not in arg) and ('.' not in arg):
        # arg is not nested.
        if arg:
            if index is not None:
                return getattr(msg, arg)[index]
            else:
                return getattr(msg, arg)
        else:
            if index is not None:
                return msg[index]
            else:
                return msg
    if '@' in arg:
        first_args, last_arg = arg.rsplit('@', 1)
    else:
        first_args, last_arg = arg.rsplit('.', 1)
    if index is not None:
        return getattr(getMsgArgValue(msg, first_args), last_arg)[index]
    else:
        return getattr(getMsgArgValue(msg, first_args), last_arg)


def setMsgArgValue(msg, arg, value, index=None):
    """Set the value of msg.arg, where arg can be a nested arg

    Set the value of msg.arg, where arg can be a nested arg. For
    example, with odom = nav_msgs.msg.Odometry() the following are valid:
        getMsgArgValue(msg, 'pose') <==> msg.pose
        getMsgArgValue(msg, 'pose.pose.position.x') <==>msg.pose.pose.position.x
    The separator in arg can be either '.' or '@', so that
        getMsgArgValue(msg, 'pose@pose@position@x') <==>msg.pose.pose.position.x
    The return value is msg.
    If arg is the empty string and msg is not a list, returns value and do
    *not* modify msg.
    If index is not None, the corresponding item list is set instead of the
    whole list.

    Parameters
    ----------
    - msg: instance of ROS message.
    - arg: str, argument to get from msg. Valid values for arg are given by
        roslib.message.get_printable_message_args(msg).split().
    - value: value to assign to msg.arg.
    - index: if not None, msg.arg[index] is assigned rather than msg.arg.
        Defaults to None.
    """
    if ('@' not in arg) and ('.' not in arg):
        # arg is not nested.
        if arg:
            if index is not None:
                setListItem(getattr(msg, arg), index, value)
            else:
                setattr(msg, arg, value)
            return msg
        else:
            if isinstance(msg, list) and index is not None:
                setListItem(msg, index, value)
                return msg
            elif isinstance(msg, list):
                for i, v in enumerate(value):
                    setListItem(msg, i, v)
                for n in range(len(msg) - len(value)):
                    msg.pop()
                return msg
            else:
                return value
    if '@' in arg:
        first_args, last_arg = arg.rsplit('@', 1)
    else:
        first_args, last_arg = arg.rsplit('.', 1)
    return setMsgArgValue(getMsgArgValue(msg, first_args),
                          last_arg,
                          value,
                          index)


def getMsgArgType(msg, arg):
    """Return the type of msg.arg, where arg can be a nested arg

    Return the type of msg.arg, where arg can be a nested arg. For
    example, with odom = nav_msgs.msg.Odometry() the following are valid:
        getMsgArgType(msg, 'pose') ==> 'nav_msgs/Odometry'
        getMsgArgType(msg, 'pose.pose.position.x') ==> 'float64
    The separator in arg can be either '.' or '@', so that
        getMsgArgType(msg, 'pose@pose@position@x') ==> 'float64

    Parameters
    ----------
    - msg: instance of ROS message.
    - arg: str, argument to get from msg. Valid values for arg are given by
        roslib.message.get_printable_message_args(msg).split().
    """
    if ('@' not in arg) and ('.' not in arg):
        # arg is not nested.
        subtypes = getMsgSubtypes(msg)
        return subtypes[arg]
    # Go one level down and return the type.
    if '@' in arg:
        first_arg, last_args = arg.split('@', 1)
    else:
        first_arg, last_args = arg.split('.', 1)
    # Get the type of msg.first_arg.
    first_arg_msg = getMsgArgValue(msg, first_arg)
    if isinstance(first_arg_msg, Time):
        # A Time message doesn't have _type argument. The return type
        # for the two possible arguments used here (last_arg = secs or nsecs)
        # is known, so we return them (hard-coded based on Indigo version).
        return 'int32'
    subtype = first_arg_msg._type
    # Get an instance of type subtype.
    submsg = roslib.message.get_message_class(subtype)()
    return getMsgArgType(submsg, last_args)


def getMsgArgFromTable(msg, tablename, field='', seq_nums=[], has_srvname=True):
    """Return the part of a message determined by a table name and a field

    Return the part of a message determined by a table name and a field. If the
    tablename contains suffix_for_array_table, seq_nums must give the index of
    each item. There is a list in the message each time suffix_for_array_table
    is met.
    """
    if tablename.count(suffix_for_array_table) != len(seq_nums):
        raise ValueError('Non consistent sequence numbers ' +
                         '({} given, {} excepted)'.format(
                             len(seq_nums),
                             tablename.count(suffix_for_array_table)))
    if has_srvname:
        # Get the table name without leading service name.
        idx_arg_start = tablename.find('@')
        if idx_arg_start > -1:
            tablename = tablename[idx_arg_start:]
            # We need a copy because we don't want to alter the original.
            seq_nums = list(seq_nums)
        else:
            # tablename is only the service name, i.e. it is irrelevant for
            # the ROS message. Get the field directly.
            return getMsgArgValue(msg, field)

    if not seq_nums:
        if field and tablename:
            new_field = tablename + '@' + field
        else:
            new_field = tablename + field
        return getMsgArgValue(msg, new_field)

    # Pop the first sequence number and associated argument value.
    seq_num0 = seq_nums.pop(0)
    first_arg, last_args = tablename.split('@' + suffix_for_array_table, 1)
    if (not last_args) and field == suffix_for_builtin:
        # Builtin-type.
        # Remove the leading '@'.
        first_arg = first_arg[1:]
        return getMsgArgValue(msg, first_arg, index=seq_num0)
    first_arg_value = getMsgArgValue(msg, first_arg, index=seq_num0)
    return getMsgArgFromTable(msg=first_arg_value,
                              tablename=last_args,
                              field=field,
                              seq_nums=seq_nums,
                              has_srvname=False)


def setMsgArgFromTable(msg, value, tablename, field='', seq_nums=[],
                       has_srvname=True):
    """Set the part of a message determined by a table name and a field

    Return the part of a message determined by a table name and a field. If the
    tablename contains suffix_for_array_table, seq_nums must give the index of
    each item. There is a list in the message each time suffix_for_array_table
    is met.
    """
    if tablename.count(suffix_for_array_table) != len(seq_nums):
        raise ValueError('Non consistent sequence numbers ' +
                         '({} given, {} excepted)'.format(
                             len(seq_nums),
                             tablename.count(suffix_for_array_table)))
    if has_srvname:
        # Get the table name without leading service name.
        idx_arg_start = tablename.find('@')
        if idx_arg_start > -1:
            tablename = tablename[idx_arg_start:]
            # We need a copy because we don't want to alter the original.
            seq_nums = list(seq_nums)
        else:
            # tablename is only the service name, i.e. it is irrelevant for
            # the ROS message. Get the field directly.
            setMsgArgValue(msg, field, value)
            return

    if not seq_nums:
        if field and tablename:
            new_field = tablename + '@' + field
        else:
            new_field = tablename + field
        setMsgArgValue(msg, new_field, value)
        return

    # Pop the first sequence number and associated argument value.
    seq_num0 = seq_nums.pop(0)
    first_arg, last_args = tablename.split('@' + suffix_for_array_table, 1)
    if (not last_args) and field == suffix_for_builtin:
        # Builtin-type.
        # Remove the leading '@' (works also with empty string).
        first_arg = first_arg[1:]
        setMsgArgValue(msg, first_arg, value, index=seq_num0)
        return
    first_arg_value = getMsgArgValue(msg, first_arg)
    if len(first_arg_value) <= seq_num0:
        # When we encounter a short list, add value at the correct place inside
        # the list.
        setMsgArgValue(msg, first_arg, value, index=seq_num0)
        return
    setMsgArgFromTable(msg=first_arg_value[seq_num0],
                       value=value,
                       tablename=last_args,
                       field=field,
                       seq_nums=seq_nums,
                       has_srvname=False)
    return


class TypeInspector(object):
    """Inspector for a ROS message or a ROS type

    There is no way (as far as we know) to get some information or create a
    message from builtin types in ROS. This class allows it.

    Parameters
    ----------
    - msg: instance of a ROS message type or str describing a message type.
    - name: str
    """
    def __init__(self, msg, name=''):
        self.msg_class = self._get_msg_class(msg)
        if isinstance(msg, roslib.message.Message):
            type_ = msg._type
        else:
            # type_ is a str describing the type.
            type_ = msg
        self.name = name
        self.type = type_
        self.is_array = type_.endswith(']')
        self.base_type = roslib.msgs.base_msg_type(type_)
        self.is_builtin = roslib.msgs.is_builtin(type_)
        self.is_array_of_builtin = (self.is_array and
                                    roslib.msgs.is_builtin(self.base_type))
        header_types = ['Header', 'std_msgs/Header', 'roslib/Header']
        self.is_header = type_ in header_types
        self.is_message = not(self.is_builtin or self.is_array_of_builtin)
        self.fields = self._setFields()

    def _get_msg_class(self, msg):
        """Return the class generating a message of the same type

        Parameters
        ----------
        - msg: roslib.message.Message instance or str describing the type
        """
        # print(msg)
        try:
            # msg is a string describing the type.
            if isinstance(msg, str) and ('[' in msg):
                idx_bracket = msg.find('[')
                msg = msg[:idx_bracket]
            msg_class = roslib.message.get_message_class(msg)
        except ValueError:
            # Builtin or array of builtins.
            msg_class = None
            # print('ValueError')
        except TypeError:
            # Instance of roslib.message.Message.
            # print('TypeError')
            msg_class = roslib.message.get_message_class(msg._type)
        return msg_class

    def _setFields(self):
        if (self.is_builtin or self.is_array_of_builtin or self.is_header):
            return {}
        name, spec = roslib.msgs.load_by_type(self.base_type)
        return dict(zip(spec.names, spec.types))

    def get_single_args(self):
        """Return the list of arguments that are a single value

        Return the list of arguments that are a single value, i.e. a value
        that can be saved as is in the database.
        For a builtin type, return an empty list.
        """
        if self.is_array_of_builtin:
            return []
        if self.is_builtin:
            return [suffix_for_builtin]
        if self.is_header:
            return ['seq', 'stamp.secs', 'stamp.nsecs', 'frame_id']
        msg = self.msg_class()
        roslib_args = roslib.message.get_printable_message_args(msg).split()
        args = []
        for arg in roslib_args:
            if arg == 'header.stamp':
                args.append('header.stamp.secs')
                args.append('header.stamp.nsecs')
                continue
            if arg.endswith('stamp'):
                # A Header type argument with is nested will be ignored. The
                # reason, is that it can be recognized easily only with its
                # ending. 'stamp' could also be used as a normal argument, so
                # we cannot consider it as being from a header argument. If
                # exists a message with such a case, a recognition feature will
                # have to be implemented here.
                rospy.logerr('header argument as nested message argument ' +
                             'not supported')
                continue
            value = getMsgArgValue(msg, arg)
            if not isinstance(value, (list, tuple)):
                args.append(arg)
        return args

    def get_list_args(self):
        """Return the list of arguments that are arrays

        Return the list of arguments that are arrays, i.e. arguments that can
        "not* be saved as is in the database.
        For a builtin type, return an empty list.
        """
        if self.is_builtin or self.is_header:
            return []
        if self.is_array_of_builtin:
            return [suffix_for_builtin]
        msg = self.msg_class()
        roslib_args = roslib.message.get_printable_message_args(msg).split()
        args = []
        for arg in roslib_args:
            value = getMsgArgValue(msg, arg)
            if isinstance(value, (list, tuple)):
                args.append(arg)
        return args


class RosSqlTable(object):
    """Container for information on SQL table and ROS message

    Parameters
    ----------
    - name: str, name of the SQL table.
    - insp: instance of TypeInspector.
    - array: True if the SQL table represents an array.
    - parentfield: '' for the highest level of a message, parent field name
        when we deal with a field of a ROS message (this field will also be an
        array).
    - parenttable: str, or None. None for non-nested tables, str (name of parent
        table) for the table this table should refer to.
    """
    def __init__(self, name, insp, parenttable=None):
        # print('* RosSqlTable')
        # print(name)
        # print(parenttable)
        # print('*end RosSqlTable')
        self.name = name
        self.parenttable = parenttable
        self.is_array = insp.is_array

        # sqltable will contain the sqlalchemy.Table instance.
        self.sqltable = None

        self._setSqlFields()
        self._setRosFields(insp)
        # All table fields and SQL types.
        self.sqlfields = self.sqlsqlfields + self.sqlrosfields
        self.sqltypes = self.sqlsqltypes + self.sqlrostypes

    def _setSqlFields(self):
        """SQL specific fields"""
        self.sqlsqlfields = ['id']
        self.sqlsqltypes = [types.Integer]
        if self.parenttable:
            self.sqlsqlfields += ['parent_id', 'seq_num']
            self.sqlsqltypes += [types.Integer, types.Integer]
        if not self.parenttable:
            self.foreign_key = ''
            self.foreign_key_ref = None
        else:
            self.foreign_key = 'parent_id'
            self.foreign_key_ref = self.parenttable + '.id'

    def _setRosFields(self, insp):
        """ROS specific fields"""
        self.sqlrosfields = []
        self.sqlrostypes = []
        # ROS fields not included in this table.
        self.rosarrayfields = []
        self.rosarraytypes = []
        if self.is_array and not self.parenttable:
            # Base table for an array, no ROS field needed.
            self.rosbasetype = None
            return

        self.rosbasetype = insp.base_type

        if insp.is_builtin or insp.is_array_of_builtin:
            self.sqlrosfields.append(rosFieldToSql(suffix_for_builtin))
            self.sqlrostypes.append(type_map[insp.base_type])
            return

        msg = insp.msg_class()
        # Single value arguments will belong to this table.
        for arg in insp.get_single_args():
            self.sqlrosfields.append(rosFieldToSql(arg))
            self.sqlrostypes.append(type_map[getMsgArgType(msg, arg)])

        # List arguments will belong to another table.
        for arg in insp.get_list_args():
            self.rosarrayfields.append(rosFieldToSql(arg))
            self.rosarraytypes.append(getMsgArgType(msg, arg))


def rosFieldToSql(rosfield):
    """Return the SQL table column name from a ROS field name

    Return the SQL table column name from a ROS field name. SQL column names
    with a period ('.') make problem to sqlalchemy.
    """
    return rosfield.replace('.', '@')


def sqlFieldToRos(sqlfield):
    """Return the ROS field name from an SQL table column name"""
    sqlfield = sqlfield.replace('@', '.')
    if sqlfield.endswith(suffix_for_builtin):
        # Get the ROS field name, i.e. everything before
        # suffix_for_builtin or ('@' + suffix...).
        return sqlfield.rsplit('@', 1)[0].replace(suffix_for_builtin, '')
    else:
        return sqlfield


class SqlMsg(object):
    """Interface between a ROS type and a database

    Parameters
    ----------
    - srv_name: str, name of the service to create table for.
    - type_: str, ROS type associated with this service.
    """
    def __init__(self, interface_name, type_):
        self.interface_name = interface_name
        self.type = type_
        self.tables = []

    def createAllTables(self, metadata):
        """Add appropriate tables to represent a ROS message in the database"""
        self._createTable(self.interface_name)
        for rossqltable in self.tables:
            table = sqlalchemy.Table(rossqltable.name, metadata)
            rossqltable.sqltable = table
            fields = rossqltable.sqlfields
            types = rossqltable.sqltypes
            for field, type_ in zip(fields, types):
                column = sqlalchemy.Column(field, type_)
                if field == 'id':
                    column.primary_key = True
                if field in rossqltable.sqlsqlfields:
                    column.nullable = False
                table.append_column(column)
                if field == rossqltable.foreign_key:
                    ref = rossqltable.foreign_key_ref
                    foreign = sqlalchemy.ForeignKeyConstraint([field], [ref])
                    table.append_constraint(foreign)

    def _createTable(self, tablename, type_=None, parenttable=None):
        """Generate the table description from message type

        For arrays, two tables are actually created and this function is called
        recursively to create a new table for the vector data.

        Parameters:
            - metadata: instance of sqlalchemy.Metadata bound to an engine.
            - type_: str, type description.
            - parent: str, name of parent table for nested tables. A nested
                table is created when an array is met in the ROS type.
                Defaults to None, which is to be used only the first time
                generateTable is called.
        """
        if type_ is None:
            insp = TypeInspector(self.type)
        else:
            insp = TypeInspector(type_)

        table = RosSqlTable(tablename,
                            insp=insp,
                            parenttable=parenttable)
        self.tables.append(table)

        if (not parenttable) and table.is_array:
            insp = TypeInspector(insp.base_type)
            nested_tablename = tablename + '@' + suffix_for_array_table
            table = RosSqlTable(nested_tablename,
                                insp=insp,
                                parenttable=tablename)
            self.tables.append(table)
        else:
            nested_tablename = tablename

        # Add the nested tables for arrays inside the message.
        for field, subtype in zip(table.rosarrayfields, table.rosarraytypes):
            subtablename = (nested_tablename +
                            '@' + field +
                            '@' + suffix_for_array_table)
            self._createTable(subtablename,
                              type_=subtype,
                              parenttable=nested_tablename)

    def setter(self, engine, msg):
        """Add to the database and return the id

        Parameters
        ----------
        - engine: sqlalchemy.engine instance.
        - msg: instance of ROS message.
        """
        def parentSeqs(ids, name):
            seqs = []
            for this_name, this_seq in ids.iterkeys():
                if this_name == name:
                    seqs.append(this_seq)
            return seqs

        def addRosFields(insert_args, msg, rossqltable, seq_nums):
            for field in rossqltable.sqlrosfields:
                insert_args[field] = getMsgArgFromTable(msg,
                                                        rossqltable.name,
                                                        field=field,
                                                        seq_nums=seq_nums)

        def insertInTable(msg, rossqltable):
            insert_args = {}
            parent_seqs = parentSeqs(ids, rossqltable.parenttable)
            for seq in parent_seqs:
                # Remove the last suffix_for_array_table
                suffix = '@' + suffix_for_array_table
                if suffix in rossqltable.name:
                    name, _ = rossqltable.name.rsplit(suffix,
                                                      1)
                else:
                    name = rossqltable.name
                parentmsg = getMsgArgFromTable(msg,
                                               name,
                                               seq_nums=seq)
                for seq_num in range(len(parentmsg)):
                    insert_args['parent_id'] = ids[rossqltable.parenttable,
                                                   seq]
                    insert_args['seq_num'] = seq_num
                    new_seq = seq + (seq_num,)
                    addRosFields(insert_args, msg, rossqltable, new_seq)
                    result = connection.execute(rossqltable.sqltable.insert(),
                                                insert_args)
                    ids[rossqltable.name, new_seq] = (
                        result.inserted_primary_key[0])

        connection = engine.connect()
        transaction = connection.begin()

        # ids is a map (table, seq_nums): id. seq_nums is the tuple of sequence
        # numbers used to reach the item with id id in the table it is found.
        ids = {}

        # Treat the first table separately.
        rossqltable = self.tables[0]
        insert_args = {}
        for field in rossqltable.sqlrosfields:
            insert_args[field] = getMsgArgFromTable(msg,
                                                    rossqltable.name,
                                                    field)
        result = connection.execute(rossqltable.sqltable.insert(), insert_args)
        return_id = result.inserted_primary_key[0]
        ids[rossqltable.name, tuple()] = return_id

        # Treat other tables.
        for rossqltable in self.tables[1:]:
            insertInTable(msg, rossqltable)

        transaction.commit()
        connection.close()
        return return_id

    def getter(self, engine, id_):
        """Retrieve a message from the database

        Parameters
        ----------
        - engine: sqlalchemy.engine instance.
        """
        def parentSeqs(ids, name):
            seqs = []
            for this_name, this_seq in ids.iterkeys():
                if this_name == name:
                    seqs.append(this_seq)
            return seqs

        def setRosFields(msg, sqlresult, rossqltable, seq_nums):
            for field in rossqltable.sqlrosfields:
                print(msg)
                print(rossqltable.name)
                print(field)
                print(seq_nums)
                setMsgArgFromTable(msg,
                                   sqlresult[field],
                                   rossqltable.name,
                                   field=field,
                                   seq_nums=seq_nums)

        def getFromTable(msg, rossqltable):
            sqltable = rossqltable.sqltable
            parent_seqs = parentSeqs(ids, rossqltable.parenttable)
            for seq in parent_seqs:
                parent_id = ids[rossqltable.parenttable, seq]
                query = sqltable.select(
                    whereclause=(sqltable.c.parent_id == parent_id))
                # print(query)
                results = connection.execute(query).fetchall()
                if not results:
                    # parent_id is not a valid parent index, this means that
                    # the array is empty and that we can skip this parent_id.
                    rospy.logdebug('Empty field: {}'.format(rossqltable.name))
                    continue
                for result in results:
                    id_ = result['id']
                    seq_num = result['seq_num']
                    new_seq = seq + (seq_num,)
                    ids[rossqltable.name, new_seq] = id_
                    insp = TypeInspector(rossqltable.rosbasetype)
                    if insp.is_message:
                        # Create a new message and add it to the list.
                        setMsgArgFromTable(msg=msg,
                                           value=insp.msg_class(),
                                           tablename=rossqltable.name,
                                           seq_nums=new_seq)
                    setRosFields(msg, result, rossqltable, new_seq)

        # ids is a map (table, seq_nums): ids.
        ids = {}

        connection = engine.connect()
        transaction = connection.begin()

        # The first table is special and needs to be treated outside the loop.
        # For example, it does not have 'parent_id' and 'seq_num' fields.
        rossqltable = self.tables[0]
        sqltable = rossqltable.sqltable
        query = sqltable.select(whereclause=(sqltable.c.id == id_))
        result = dict(connection.execute(query).fetchone())
        if result is None:
            rospy.logerr('No id {} in table {}'.format(id_, rossqltable.name))
            transaction.commit()
            connection.close()
            return
        ids[rossqltable.name, tuple()] = id_

        if rossqltable.is_array:
            msg = []
        else:
            insp = TypeInspector(rossqltable.rosbasetype)
            # The first table cannot be a builtin type, so insp.msg_class
            # is valid.
            msg = insp.msg_class()
            for field in rossqltable.sqlrosfields:
                setMsgArgValue(msg, field, result[field])

        # All other tables are arrays.
        for rossqltable in self.tables[1:]:
            getFromTable(msg, rossqltable)

        transaction.commit()
        connection.close()
        return msg


class DBInterface(DBInterfaceAbstract):
    @property
    def interface_type(self):
        return "cleartext"

    def setter(self, msg):
        """Execute the setter service and return the reponse"""
        # Create an instance of response.
        response = self.setter_service_class._response_class()
        id_ = self.sqlmsg.setter(self.engine, msg.descriptor)
        # Return the descriptor identifier.
        response.id.descriptor_id = id_
        response.id.interface_name = self.interface_name
        return response

    def getter(self, msg):
        """Execute the getter service and return the response"""
        # Create an instance of response.
        response = self.getter_service_class._response_class()
        result = self.sqlmsg.getter(self.engine, msg.id.descriptor_id)
        response.descriptor = result
        return response

    def _generateSchema(self):
        """Generate schema from response class

        Recusrively generate tables for the type of the 'descriptor' variable
        in the response class.
        """
        response = self.getter_service_class._response_class()
        slots = getMsgSubtypes(response)
        try:
            type_ = slots['descriptor']
        except KeyError:
            rospy.logfatal('Getter service has no "descriptor" field in its' +
                           'reponse')
            return

        # Add the type description.
        self._addInterfaceDescription()

        self.sqlmsg = SqlMsg(interface_name=self.interface_name, type_=type_)
        self.sqlmsg.createAllTables(self.metadata)

        # Create the tables in database.
        self.metadata.create_all()


def cleartext_interface_factory(interface_name, getter_srv_msg, setter_srv_msg):
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
