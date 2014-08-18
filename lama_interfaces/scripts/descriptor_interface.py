#!/usr/bin/env python
import roslib; roslib.load_manifest('lama_interfaces')
import rospy
import roslib.msgs
import roslib.message
#import genmsg
import std_msgs.msg
import lama_interfaces.msg
from lama_interfaces.msg import LamaMapAction
from lama_interfaces.msg import LamaDescriptorIdentifier

from sqlalchemy import *
import sqlalchemy.types as s


## mapping from python types to sqlalchemy types 
type_map = {
      'bool':
      s.Boolean(),
      'char':
      s.SmallInteger(unsigned=True),
      'int8':
      s.SmallInteger(),
      'uint8':
      s.SmallInteger(unsigned=True),
      'byte':
      s.SmallInteger(),
      'int16':
      s.Integer(),
      'uint16':
      s.Integer(unsigned=True),
      'int32':
      s.Integer(),
      'uint32':
      s.Integer(unsigned=True),
      'int64':
      s.BigInteger(),
      'uint64':
      s.BigInteger(unsigned=True),
      'float32':
      s.Float(precision=32),
      'float64':
      s.Float(precision=64),
      'string':
      s.Text(),
      }

## create engine 
## TODO according  the rosparam server

engine = create_engine('sqlite:///created.sql')
class Struct(object):
    def __init__(self, adict):
        """Convert a dictionary to a class

        @param :adict Dictionary
        """
        self.__dict__.update(adict)
        for k, v in adict.items():
           if isinstance(v, dict):
              self.__dict__[k] = Struct(v)

def get_object(adict):
   """Convert a dictionary to a class

   @param :adict Dictionary
   @return :class:Struct
   """
   return Struct(adict)


class Lama_descriptor_class:

  
   def pull_vertex(self, descId):
      print 'pull ', descId.object_id
      table = self.metadata.tables['lama_descriptors']
      query = table.select()
      query = query.where(table.c['object_id'] == descId.object_id)
      con = engine.connect()
      result =  con.execute(query).fetchAll()
      con.close()
      # check if something returned
      response = self._action_class._response_class()
      iid= result.id
#      print "getter return ",ret,ret.__class__
      response.objects.append(self._get_object(iid))
      return response

   def get_descriptor_id (self, descId):
      
      table = self.metadata.tables['lama_descriptors']
      query = select([lama_descriptorstable.c.descriptor_id], from_obj=[table])
      query = query.where(reftable.c['object_id'] == descId.object_id)
      print query
      con = engine.connect()
      result =  con.execute(query).fetchall()
      con.close()
      # check if something returned
      print result
      for r in result:
	 print r.description_id
         response.append(self._get_object(r.descriptor_id))
      return response 

   
   def __init__(self):
      self.metadata = MetaData()			# sqlalchemy metadata holds the table definition
      self.ident = 1 	

   def init_ident(self):
      query = select([func.max(self.metadata.tables['lama_descriptors'].c.id)], from_obj=[self.metadata.tables['lama_descriptors']])
      print query
      con = engine.connect()
      result =  con.execute(query).fetchone()
      con.close()
      # check if something returned
      if result.max_1 is None:
	 print "none"
	 self.ident = 1
      else:
	 print result
	 self.ident = result.max_1
  

   def setter(self,):      
      self.ident = self.ident+1 			# autoincrement 
      con = engine.connect()
      self.insert_into_sql()		# insert dictionary to sql
      con.close()
      return r 
   
   def getter(self,msg):
      return ret 

   ## get columns from table (table_name) and identifier  ident  
   def getColumns(self, table_name, ident):
      table = self.metadata.tables[table_name]
      query = table.select()
      query = query.where(table.c.id == ident)
      ###print query
      con = engine.connect()
      result =  con.execute(query).fetchone()
      con.close()
     ###print "one result" , result
#      if result:
#	 return dict(zip(table.c.keys(),result)) 		# convert query result to dictionary
#      return table.c.keys();
      return dict(zip(table.c.keys(),result)) 		# convert query result to dictionary

   ## get columns from table (table_name) and identifier  ident in case the table represent an array 
   def getArrayColumns(self, table_name, ident):
      table = self.metadata.tables[table_name]
      query = table.select()
      query = query.where(table.c.parent_id == ident )
      query = query.order_by(table.c.seq_num)
      ###print query
      con = engine.connect()
      result =  con.execute(query).fetchall()
      con.close()
      ###print table.c.keys()
      return  [dict(zip(table.c.keys(), r))  for r in result]		# convert query result to array of dictionaries 

   ## generate schema from response class 
   def generateSchema(self):
      self.generateTable()
      for t in self.metadata.sorted_tables:
	 print "=== table ===" 
	 print t.name
	 print "-------------" 
	 for c in t.c:
	    print c
         print "============"
      self.metadata.create_all(engine) # create table in database 

   ## generate table description from class (cls)  for array generate new table  recursively called 
   def generateTable(self):
	table = Table("lama_descriptors", self.metadata, Column('id',Integer, primary_key=True))
	table.append_column(Column('object_id', Integer))
	table.append_column(Column('descriptor_id', Integer))
	table.append_column(Column('interface_name',String))
	print table

if __name__ == '__main__':
   ld = Lama_descriptor_class()
   ld.generateSchema()
   ld.init_ident()
