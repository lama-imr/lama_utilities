#include <string>

#include <sqlite3.h>

#include <ros/ros.h>

#include <lama_interfaces/lama_utils.h>


int getDescriptorId(sqlite3* db, int object_id, std::string interface_name)
{
  sqlite3_stmt *statement;
  int result = 0;
  int descriptor_id = 0;
  ROS_DEBUG("descriptor get");
  if (sqlite3_prepare_v2(db, "select descriptor_id from view  where vertex_id = ? and interface_name = ? ;", -1, &statement, 0) == SQLITE_OK)
  {
    sqlite3_bind_int(statement, 1, object_id);
    sqlite3_bind_text(statement, 2, interface_name.c_str(), -1, SQLITE_TRANSIENT);

    if ((result = sqlite3_step(statement)) == SQLITE_ROW)
    {
      descriptor_id = sqlite3_column_int (statement,0);
      ROS_DEBUG("object id %i -> descritor id  %i", object_id,descriptor_id);
    }
    else
    {
      ROS_ERROR ("Cannot get descriptor object %i interface %s", object_id, interface_name.c_str());
      descriptor_id = -1;
    }

  }
  else
  {
    ROS_ERROR("LAMACORE cannot prepare sqlite3 get descriptor statement");
    descriptor_id = -1;
  }

  sqlite3_finalize(statement);
  ROS_DEBUG("descriptor get finalize");
  return descriptor_id;
};

