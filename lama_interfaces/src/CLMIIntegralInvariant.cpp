#include <string>

#include "ros/ros.h"

#include <lama_interfaces/lama_utils.h>
#include <lama_interfaces/CLMIIntegralInvariant.h>

CLMIIntegralInvariant::CLMIIntegralInvariant(std::string map_file)
{
  map = map_file;
}

bool CLMIIntegralInvariant::getter(lama_interfaces::lmi_vector_double_get::Request& req, lama_interfaces::lmi_vector_double_get::Response& res)
{
  bool ret = false;
  sqlite3_stmt *statement;
  int result = 0;
  int rowCount = 0;
  if (req.id.descriptor_id <0)
  {
    req.id.descriptor_id = getDescriptorId(db,req.id.object_id, req.id.interface_name);
  }

  if(sqlite3_prepare_v2(db, "select value, sequence_number  from integral_invariant_coefficient where id = ? order by sequence_number asc ;", -1, &statement, 0) == SQLITE_OK)
  {
    sqlite3_bind_int(statement,1,req.id.descriptor_id);
      
    while ((result = sqlite3_step(statement)) == SQLITE_ROW)
    {
      rowCount++;
      res.descriptor.push_back( sqlite3_column_double(statement, 0));
    } 
    ret = rowCount;

  }
  else
  {
    ROS_ERROR("cannot prepare moment getter statement");
    ret =  false;
  }
  sqlite3_finalize(statement);
  return ret;
};

bool CLMIIntegralInvariant::setter(lama_interfaces::lmi_vector_double_set::Request& req, lama_interfaces::lmi_vector_double_set::Response& res)
{
  bool ret = false;
  sqlite3_stmt *statement;
  sqlite3_stmt *statement_max_id;
  int result = 0;
  int actual_id = 0;
  int err;
  ROS_DEBUG("integral invariants setter");

  //  ros::param::param<int>("lama_actual_view", actual_id, 1);

  if ((err = sqlite3_prepare_v2(db, "select max(id) from integral_invariant_coefficient;",-1,&statement_max_id,0)) == SQLITE_OK)
  {
    result =sqlite3_step(statement_max_id);
    if (result == SQLITE_ROW)
    {
      actual_id = sqlite3_column_int(statement_max_id,0)+1;
    }
    else
    {
      ROS_ERROR ("failed get max id i %i",result);
    }
  }
  else
  {
    ROS_ERROR("cannot prepare sqlite3 statement select max: %i", err);
    ret =  false;
  }
  sqlite3_finalize(statement_max_id);

  if (sqlite3_exec(db, "BEGIN IMMEDIATE TRANSACTION;", 0, 0, 0) != SQLITE_OK)
    ROS_ERROR("begin transaction failed");
  if ((err = sqlite3_prepare_v2(db, "insert into integral_invariant_coefficient (id, sequence_number, value) values (?, ?, ?);", -1, &statement, 0)) == SQLITE_OK)
  {
    for (uint i = 0 ; i < req.descriptor.size(); i++)
    {
      sqlite3_bind_int(statement,1,actual_id);
      sqlite3_bind_int(statement,2,i);
      sqlite3_bind_double(statement,3,req.descriptor[i]);
      ROS_DEBUG("iset moment %i %f",i,req.descriptor[i]);
      result = sqlite3_step(statement);
      sqlite3_reset(statement);
      sqlite3_clear_bindings(statement);

      if (result == SQLITE_DONE)
      {
      }
      else
      {
        ROS_ERROR("failed insert zerike moment %i: %i",i,result);
        ret = false;
      }
    }

    if (result == SQLITE_DONE)
    {
      res.id.descriptor_id = actual_id;
      ROS_DEBUG("integral invariants finalize with :%i", sqlite3_get_autocommit(db));
      ret = true;
    }
    else
    {
      ROS_ERROR("failed insert integral invariants: %i",result);
      ret = false;
    }
    if (sqlite3_exec(db, "COMMIT;", 0, 0, 0) != SQLITE_OK)
    {
      ROS_ERROR ("COMMIT FAILED");
    }
  }
  else
  {
    ROS_ERROR("cannot prepare sqlite3 statement insert int: %i", err);
    ret = false;
  }
  sqlite3_finalize(statement);
  return ret;
};

bool CLMIIntegralInvariant::connect()
{
  int rc;

  rc = sqlite3_open(map.c_str(), &db);
  if (rc)
  {
    ROS_ERROR("Can't open database: %s\n", sqlite3_errmsg(db));
    sqlite3_close(db);
    return false;
  }
  /*   rc = sqlite3_exec(db, "create table if not exists integral_invariant ( id int, view int); ", 0, 0, 0);
       if( rc ){
       ROS_ERROR("Can't create table: %s\n", sqlite3_errmsg(db));
       sqlite3_close(db);
       return false;
       }
       */
  rc = sqlite3_exec(db, "create table if not exists integral_invariant_coefficient ( id int, sequence_number int ,  value double);", 0, 0, 0);
  if (rc)
  {
    ROS_ERROR("Can't create table: %s\n", sqlite3_errmsg(db));
    sqlite3_close(db);
    return false;
  }
  return true;
}
